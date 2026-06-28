#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "EVE.h"
#include "EVE_commands.h"
#include "esp_task_wdt.h"

#include "mqtt_client.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_crt_bundle.h"

#include "cJSON.h"

#include "batteries_gps_motor_2.h"  // FF DTU Boat CAN Library
#include "nma2000_motor.h"
#include "motor_estimated.h"  // NMA2000 Motor CAN Libraty calibrated
#include "gvi_motor.h"
#include "dht_lv_batt.h"
#include "driver/twai.h"
#include "batteries.h"
// can pins
#define CAN_TX  GPIO_NUM_41
#define CAN_RX  GPIO_NUM_42

//WIFI credentials
#define WIFI_SSID "DTU_FF_Boat_Team"
#define WIFI_PASS "DTUBoatFF2425#"
static int maxretries = 207;

// for the mqtt messages
#define RX_TOPIC_MAX   128
#define RX_PAYLOAD_MAX 256

// for the button
#define BTN_PIN GPIO_NUM_40
static volatile uint8_t current_screen = 0;

#define CRUISE_BTN_PIN  GPIO_NUM_39   
#define CRUISE_RPM      400           
#define CRUISE_CAN_ID   0x300

//mqtt global client
static esp_mqtt_client_handle_t mqtt_client;

static const char *LOG_TAG = "DASH";

// macros used in the drawings
#define PI              3.14159265f
#define MAX_POWER_KW    30f
#define MAX_RPM         4000.0f
#define MAX_SPD         50.0f
#define BAT_TEMP_WARN   
#define BAT_TEMP_ERROR  60
#define HUM_WARN        75

#define CX  400
#define CY  220


//---- Data Structure for the queues ----------------------

typedef struct {
    uint16_t rpm;
    uint16_t voltagedc;
    int16_t currentdc;
    int16_t temp;
    float    speed_kn;
    float    power_kw;
} motor_data_t;

typedef struct {
    uint8_t  soc;
    float    voltage;
    float    current;
    uint8_t  temp_c;
} battery_t;

typedef struct {
    battery_t bat[3];
} battery_pack_t;

typedef struct {
    uint8_t humidity;
} sensor_data_t;

typedef struct {
    float range_min_kw;
    float range_max_kw;
    bool  active;
} power_window_t;

#define MQTT_MSG_MAX_LEN  80
#define MQTT_MSG_MAX_ROWS 3

typedef struct {
    char     lines[MQTT_MSG_MAX_ROWS][MQTT_MSG_MAX_LEN];
    uint8_t  count;
} mqtt_messages_t;

//----- Code Start --------------------------------------------------

static QueueHandle_t q_motor;
static QueueHandle_t q_battery;
static QueueHandle_t q_sensor;
static QueueHandle_t q_power_window;
static QueueHandle_t q_mqtt_msgs;

//----- WiFi and MQTT initilization ---------------------------------
static bool wifi_connected = false;
static int retry_count = 0;

static esp_timer_handle_t wifi_reconnect_timer;

// callback del timer: gira nel task esp_timer, NON nell'event loop
static void wifi_reconnect_cb(void *arg)
{
    esp_wifi_connect();   // non bloccante, va benissimo qui
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED && maxretries > 0) {
        wifi_connected = false;
        if (retry_count < maxretries) {
            retry_count++;
            // backoff con tetto: 500ms * n, ma max 8 secondi
            uint32_t delay_ms = 500 * retry_count;
            if (delay_ms > 8000) delay_ms = 8000;
            // riarmo il timer (lo stoppo prima per sicurezza)
            esp_timer_stop(wifi_reconnect_timer);
            esp_timer_start_once(wifi_reconnect_timer, (uint64_t)delay_ms * 1000);
        }
        //wifi_connected = false;
        /*if (retry_count < maxretries) {
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(500 * retry_count));  // backoff crescente
            esp_wifi_connect();
        }*/
        //esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        retry_count = 0;
    }
}

static bool mqtt_connected = false;

//mqtt events handler

// message format: -powerMin, powerMax --> json, text-->string
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;


    switch (event->event_id) {

        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(mqtt_client, "boat/message/#", 1);
            mqtt_connected = true;
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            break;

        case MQTT_EVENT_DATA: {
            // copy topic
            char topic[RX_TOPIC_MAX];
            int tlen = (event->topic_len < (int)sizeof(topic) - 1) ? event->topic_len : (int)sizeof(topic) - 1;
            memcpy(topic, event->topic, tlen);
            topic[tlen] = '\0';

            // copy payload
            char payload[RX_PAYLOAD_MAX];
            int plen = (event->data_len < (int)sizeof(payload) - 1) ? event->data_len : (int)sizeof(payload) - 1;
            memcpy(payload, event->data, plen);
            payload[plen] = '\0';

            if (strcmp(topic, "boat/message/text") == 0) {

                // check the current status of the queue
                mqtt_messages_t msgs;
                if (xQueuePeek(q_mqtt_msgs, &msgs, 0) != pdTRUE) {
                memset(&msgs, 0, sizeof(msgs));
                }

                // shift to make room for the new message
                for (int i = MQTT_MSG_MAX_ROWS - 1; i > 0; i--) {
                    strncpy(msgs.lines[i], msgs.lines[i - 1], MQTT_MSG_MAX_LEN - 1);
                    msgs.lines[i][MQTT_MSG_MAX_LEN - 1] = '\0';
                }

                // new message on top
                strncpy(msgs.lines[0], payload, MQTT_MSG_MAX_LEN - 1);
                msgs.lines[0][MQTT_MSG_MAX_LEN - 1] = '\0';
                if (msgs.count < MQTT_MSG_MAX_ROWS) msgs.count++;

                xQueueOverwrite(q_mqtt_msgs, &msgs);
                
            } else if (strcmp(topic, "boat/message/power") == 0) {

                cJSON *root = cJSON_Parse(payload);
                if (root == NULL) {
                    ESP_LOGE("MQTT", "JSON parse failed");
                    break;
                }

                cJSON *min = cJSON_GetObjectItem(root, "powerMin");
                cJSON *max = cJSON_GetObjectItem(root, "powerMax");

                if (cJSON_IsNumber(min) && cJSON_IsNumber(max)) {
                    if (min->valuedouble == 0 && max->valuedouble == 0){
                        power_window_t pw = {
                        .range_min_kw = 0,
                        .range_max_kw = 0,
                        .active       = false
                        };
                        xQueueOverwrite(q_power_window, &pw);
                        cJSON_Delete(root);  
                        break;
                    }
                    power_window_t pw = {
                        .range_min_kw = (float)min->valuedouble,
                        .range_max_kw = (float)max->valuedouble,
                        .active       = true
                    };
                    xQueueOverwrite(q_power_window, &pw);
                }

                cJSON_Delete(root);  
            }
            break;  
        }           

        case MQTT_EVENT_ERROR:
            ESP_LOGE("MQTT", "MQTT_EVENT_ERROR");
            break;

        default:
            break;
    }
}


//initialization function for mqtt connection
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = "mqtts://fbe643f0aede453b8a874f9eef1b696b.s1.eu.hivemq.cloud:8883",
    .credentials.client_id = "ESP32Telemetry2",
    .credentials.username = "ffServer",
    .credentials.authentication.password = "BoatServer25",
    .broker.verification.use_global_ca_store = false,
    .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}


//initialization function for wi-fi
void wifi_init_sta(void)
{
    // Start NVS
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Handler 
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,&wifi_event_handler, NULL, NULL);

    //SSID and password
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    const esp_timer_create_args_t targs = {
    .callback = wifi_reconnect_cb,
    .name     = "wifi_reconnect"
    };
    esp_timer_create(&targs, &wifi_reconnect_timer);
    esp_wifi_start();

    //printf("Wi-Fi started, conntection to SSID: %s\n", WIFI_SSID);
}

//------ WIfi and MQTT status symbols on the screen ------------------

static void draw_connectivity_status(bool wifi_ok, bool mqtt_ok)
{
    const int16_t BY = 460;   

    // WiFi
    EVE_color_rgb_burst(wifi_ok ? 0x00CC00UL : 0xFF2200UL);
    EVE_cmd_text_burst(670, BY, 23, EVE_OPT_CENTER, "WiFi");

    // separator 
    EVE_color_rgb_burst(0x333333UL);
    EVE_line_width_burst(1 * 16);
    EVE_begin_burst(EVE_LINES);
    EVE_vertex2f_burst(724 * 16, (BY - 8) * 16);
    EVE_vertex2f_burst(724 * 16, (BY + 8) * 16);
    EVE_end_burst();

    // MQTT
    EVE_color_rgb_burst(mqtt_ok ? 0x00CC00UL : 0xFF2200UL);
    EVE_cmd_text_burst(750, BY, 23, EVE_OPT_CENTER, "MQTT");
}

// ----- Colours for the main values --------------------------------

static uint32_t soc_color(uint8_t soc)
{
    if (soc <= 15U) return 0xFF2200UL;
    if (soc <= 30U) return 0xFFAA00UL;
    return 0x00CC00UL;
}

static uint32_t temp_color(uint8_t temp)
{
    if (temp >= BAT_TEMP_ERROR) return 0xFF2200UL;
    if (temp >= BAT_TEMP_WARN)  return 0xFFAA00UL;
    return 0x00AA00UL;
}

// ------- Draw Arc for the power gauge -----------------------------
// radiants, start=150° sweep=240°

static void draw_arc_segment(int16_t cx, int16_t cy, uint16_t radius, float val_start, float val_end, float val_max,uint32_t color, uint16_t thickness)//center (x, y) 
{
    float sweep     = 240.0f * PI / 180.0f;
    float ang_start = 150.0f * PI / 180.0f;
    float a0 = ang_start + sweep * (val_start / val_max);
    float a1 = ang_start + sweep * (val_end   / val_max);

    int steps = (int)((a1 - a0) / (PI / 180.0f));
    if (steps < 2) steps = 2;

    EVE_color_rgb_burst(color);
    EVE_line_width_burst(thickness);
    EVE_begin_burst(EVE_LINE_STRIP);
    for (int i = 0; i <= steps; i++) {
        float a = a0 + (a1 - a0) * i / (float)steps;
        int16_t x = cx + (int16_t)((float)radius * cosf(a));
        int16_t y = cy + (int16_t)((float)radius * sinf(a));
        EVE_vertex2f_burst(x * 16, y * 16);
    }
    EVE_end_burst();
}

// ----- RPM bar ------------------------------------------------------

static void draw_rpm_bar(uint16_t rpm, uint16_t max, const char * label)
{
    int16_t  bx = 0;
    int16_t  by = 0;
    uint16_t bw = 800;
    uint16_t bh = 28;

    // background
    EVE_color_rgb_burst(0x1a1a1aUL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(bx * 16,        by * 16);
    EVE_vertex2f_burst((bx+bw) * 16,   (by+bh) * 16);
    EVE_end_burst();

    // colour zones: green 0-65%, yellow 65-85%, red 85-100%
    uint16_t green_w  = (uint16_t)(bw * 65 / 100);
    uint16_t yellow_w = (uint16_t)(bw * 85 / 100);

    EVE_color_rgb_burst(0x006600UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(bx * 16,               by * 16);
    EVE_vertex2f_burst((bx + green_w) * 16,   (by+bh) * 16);
    EVE_end_burst();

    EVE_color_rgb_burst(0x886600UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst((bx + green_w) * 16,   by * 16);
    EVE_vertex2f_burst((bx + yellow_w) * 16,  (by+bh) * 16);
    EVE_end_burst();

    EVE_color_rgb_burst(0x880000UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst((bx + yellow_w) * 16,  by * 16);
    EVE_vertex2f_burst((bx + bw) * 16,        (by+bh) * 16);
    EVE_end_burst();

    // dark mask to cover until the rpm value
    if (rpm > max) rpm = max;
    uint16_t fill_w = (uint16_t)((float)bw * rpm / max);
    if (fill_w < bw) {
        EVE_color_rgb_burst(0x0a0a0aUL);
        EVE_begin_burst(EVE_RECTS);
        EVE_vertex2f_burst((bx + fill_w) * 16,  by * 16);
        EVE_vertex2f_burst((bx + bw) * 16,      (by+bh) * 16);
        EVE_end_burst();
    }

    // valore RPM centrato
    EVE_color_rgb_burst(0xffffffff);
    EVE_cmd_number_burst(400, 14, 30, EVE_OPT_CENTER, (int32_t)rpm);
    EVE_color_rgb_burst(0xaaaaaaUL);
    EVE_cmd_text_burst(460, 14, 28, EVE_OPT_CENTERY, label);
}

// ----- Power Gauge --------------------------------------------------

// function for drawing ticks on the arc
static void draw_gauge_ticks_and_labels(uint16_t radius, float val_max, int tick_step, const power_window_t *pw)
{
    float sweep    = 240.0f * PI / 180.0f;
    float ang_base = 150.0f * PI / 180.0f;
    uint16_t rlab  = radius + 24;  // raggio etichette

    int n_ticks = (int)(val_max / tick_step);

    for (int i = 0; i <= n_ticks; i++) {
        float val = i * tick_step;
        float a   = ang_base + sweep * (val / val_max);

        // tick lungo
        int16_t x1 = CX + (int16_t)((radius - 8) * cosf(a));
        int16_t y1 = CY + (int16_t)((radius - 8) * sinf(a));
        int16_t x2 = CX + (int16_t)((radius + 8) * cosf(a));
        int16_t y2 = CY + (int16_t)((radius + 8) * sinf(a));

        EVE_color_rgb_burst(0x888888UL);
        EVE_line_width_burst(2 * 16);
        EVE_begin_burst(EVE_LINES);
        EVE_vertex2f_burst(x1 * 16, y1 * 16);
        EVE_vertex2f_burst(x2 * 16, y2 * 16);
        EVE_end_burst();

        // numero etichetta
        int16_t lx = CX + (int16_t)(rlab * cosf(a));
        int16_t ly = CY + (int16_t)(rlab * sinf(a));

        // verde se dentro il range attivo, bianco altrimenti
        bool in_range = (pw != NULL) && pw->active &&val >= pw->range_min_kw && val <= pw->range_max_kw;
        EVE_color_rgb_burst(in_range ? 0x00DD00UL : 0xFFFFFFUL);
        EVE_cmd_number_burst(lx, ly, 23, EVE_OPT_CENTER, (int32_t)val);
    }

    // tick corti intermedi
    int minor_step = tick_step / 2;
    if (minor_step > 0) {
        int n_minor = (int)(val_max / minor_step);
        for (int i = 0; i <= n_minor; i++) {
            if (i % 2 == 0) continue;
            float val = i * minor_step;
            float a   = ang_base + sweep * (val / val_max);

            int16_t x1 = CX + (int16_t)((radius - 4) * cosf(a));
            int16_t y1 = CY + (int16_t)((radius - 4) * sinf(a));
            int16_t x2 = CX + (int16_t)((radius + 4) * cosf(a));
            int16_t y2 = CY + (int16_t)((radius + 4) * sinf(a));

            EVE_color_rgb_burst(0x444444UL);
            EVE_line_width_burst(1 * 16);
            EVE_begin_burst(EVE_LINES);
            EVE_vertex2f_burst(x1 * 16, y1 * 16);
            EVE_vertex2f_burst(x2 * 16, y2 * 16);
            EVE_end_burst();
        }
    }
}

// function for actually drawing the gauge for power/speed/rpm
static void draw_gauge_generic(float power_kw, uint16_t max, const char* label, const power_window_t *pw)
{
    const uint16_t RAD = 120;
    const uint16_t THK = 14 * 16;
    const uint16_t RLAB = RAD + 28;

    // borders of the range 
    float sweep    = 240.0f * PI / 180.0f;
    float ang_base = 150.0f * PI / 180.0f;

    // main arc
    draw_arc_segment(CX, CY, RAD, 0.0f, (float)max,(float)max, 0x222222UL, THK);

    // green arc for target power range from mqtt, only if it is power
    if (strcmp(label, "kW") == 0) {
    
        if (pw != NULL && pw->active) {
            draw_arc_segment(CX, CY, RAD,pw->range_min_kw, pw->range_max_kw,MAX_POWER_KW, 0x007700UL, THK); //color green

            for (int edge = 0; edge < 2; edge++) {
                float val = (edge == 0) ? pw->range_min_kw : pw->range_max_kw;
                float a   = ang_base + sweep * (val / MAX_POWER_KW);
                int16_t x1 = CX + (int16_t)((RAD - 10) * cosf(a));
                int16_t y1 = CY + (int16_t)((RAD - 10) * sinf(a));
                int16_t x2 = CX + (int16_t)((RAD + 10) * cosf(a));
                int16_t y2 = CY + (int16_t)((RAD + 10) * sinf(a));
                EVE_color_rgb_burst(0xFFFFFFUL);
                EVE_line_width_burst(2 * 16);
                EVE_begin_burst(EVE_LINES);
                EVE_vertex2f_burst(x1 * 16, y1 * 16);
                EVE_vertex2f_burst(x2 * 16, y2 * 16);
                EVE_end_burst();
            }
            // label range
            char rng[24];
            snprintf(rng, sizeof(rng), "%.0f - %.0f kW", (double)pw->range_min_kw, (double)pw->range_max_kw);
            EVE_color_rgb_burst(0x00AA00UL);
            EVE_cmd_text_burst(CX, CY + 85, 23, EVE_OPT_CENTER, rng);
        }
        draw_gauge_ticks_and_labels(RAD, MAX_POWER_KW, 10, pw);

    }else if (strcmp(label, "knots") == 0){
        draw_gauge_ticks_and_labels(RAD, MAX_SPD, 10, NULL);
    }else if(strcmp(label, "rpm") == 0){
        draw_gauge_ticks_and_labels(RAD, MAX_RPM, 500, NULL);
    }

    // needle
    float ang_needle = ang_base + sweep * (power_kw / (float)max);
    int16_t nx = CX + (int16_t)((RAD - 10) * cosf(ang_needle));
    int16_t ny = CY + (int16_t)((RAD - 10) * sinf(ang_needle));
    EVE_color_rgb_burst(0xFFFFFFUL);
    EVE_line_width_burst(3 * 16);
    EVE_begin_burst(EVE_LINES);
    EVE_vertex2f_burst(CX * 16, CY * 16);
    EVE_vertex2f_burst(nx * 16, ny * 16);
    EVE_end_burst();

    // center circle
    EVE_color_rgb_burst(0x444444UL);
    EVE_point_size_burst(10 * 16);
    EVE_begin_burst(EVE_POINTS);
    EVE_vertex2f_burst(CX * 16, CY * 16);
    EVE_end_burst();

    // power value 
    EVE_color_rgb_burst(0xFFFFFFUL);
    char pw_str[12];
    snprintf(pw_str, sizeof(pw_str), "%.1f", (double)power_kw);
    EVE_cmd_text_burst(CX, CY + 40, 31, EVE_OPT_CENTER, pw_str);
    EVE_color_rgb_burst(0x888888UL);
    EVE_cmd_text_burst(CX, CY + 69, 26, EVE_OPT_CENTER, label);

}

// ------- Speed -----------------------------------------------

static void draw_speed(float speed_kn, const char * title, const char * label)
{
    EVE_cmd_romfont_burst(1U, 33U);
    // box speed: x=580, y=36, w=210, h=170
    EVE_color_rgb_burst(0x111111UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(580 * 16,  36 * 16);
    EVE_vertex2f_burst(790 * 16, 170 * 16);
    EVE_end_burst();

    EVE_color_rgb_burst(0x555555UL);//grey
    EVE_cmd_text_burst(685, 52, 25, EVE_OPT_CENTER, title);

    EVE_color_rgb_burst(0xFFFFFFUL); // white
    char spd[8];
    snprintf(spd, sizeof(spd), "%.1f", (double)speed_kn);
    EVE_cmd_text_burst(685, 114, 1, EVE_OPT_CENTER, spd);

    EVE_color_rgb_burst(0x888888UL); // grey
    EVE_cmd_text_burst(685, 152, 26, EVE_OPT_CENTER, label);
}

// ----------- Batteries ----------------------------------------

static void draw_battery_box(int16_t x, int16_t y, uint16_t w, uint16_t h,uint8_t idx, const battery_t *bat)
{
    bool alarm = bat->temp_c >= BAT_TEMP_ERROR; //check for the alarm

    // borders of the box
    EVE_color_rgb_burst(alarm ? 0xFF2200UL : 0x333333UL);
    EVE_line_width_burst(1 * 16);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(x * 16,         y * 16);
    EVE_vertex2f_burst((x + w) * 16,   (y + h) * 16);
    EVE_end_burst();

    // background 
    EVE_color_rgb_burst(alarm ? 0x1a0808UL : 0x111111UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst((x + 1) * 16,       (y + 1) * 16);
    EVE_vertex2f_burst((x + w - 1) * 16,   (y + h - 1) * 16);
    EVE_end_burst();

    // row 1: label + SOC (y + 20)
    char lbl[10];
    if (idx == 2){
        snprintf(lbl, sizeof(lbl), "LVB1");
    }else{
        snprintf(lbl, sizeof(lbl), "B%d", idx + 1);
    }
    //EVE_color_rgb_burst(alarm ? 0xFF4400UL : 0x555555UL); 0xFFFFFFUL
    EVE_color_rgb_burst(alarm ? 0xFF4400UL : 0xFFFFFFUL);
    EVE_cmd_text_burst(x + 8, y + 20, 25, EVE_OPT_CENTERY, lbl);

    char soc_str[8];
    snprintf(soc_str, sizeof(soc_str), "%d%%", bat->soc);
    EVE_color_rgb_burst(soc_color(bat->soc));
    EVE_cmd_text_burst(x + w - 8, y + 20, 25, EVE_OPT_RIGHTX | EVE_OPT_CENTERY, soc_str);

    // row 2: SOC bar (y + 36 -- y + 56)
    int16_t  bx = x + 8;
    int16_t  by = y + 36;
    uint16_t bw = w - 16;
    uint16_t bh = 20;
    uint16_t fw = (uint16_t)((float)bw * bat->soc / 100.0f);

    EVE_color_rgb_burst(0x1a1a1aUL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(bx * 16,          by * 16);
    EVE_vertex2f_burst((bx + bw) * 16,   (by + bh) * 16);
    EVE_end_burst();

    if (fw > 0U) {
        EVE_color_rgb_burst(soc_color(bat->soc));
        EVE_begin_burst(EVE_RECTS);
        EVE_vertex2f_burst(bx * 16,          by * 16);
        EVE_vertex2f_burst((bx + fw) * 16,   (by + bh) * 16);
        EVE_end_burst();
    }

    // row 3: voltage and temperature (y + 75)
    char volt_str[10];
    snprintf(volt_str, sizeof(volt_str), "%.1fV", (double)bat->voltage);
    EVE_color_rgb_burst(0x555555UL);
    EVE_cmd_text_burst(x + 8, y + 70, 24, EVE_OPT_CENTERY, volt_str);

    char temp_str[8];
    snprintf(temp_str, sizeof(temp_str), "%dC", bat->temp_c);
    EVE_color_rgb_burst(temp_color(bat->temp_c));
    EVE_cmd_text_burst(x + w - 8, y + 70, 24, EVE_OPT_RIGHTX | EVE_OPT_CENTERY, temp_str);
}

static void draw_batteries(const battery_pack_t *pack)
{
    // two batteries: x=580, y=178, w=210
    draw_battery_box(580, 178, 210, 86, 0, &pack->bat[0]);
    draw_battery_box(580, 268, 210, 86, 1, &pack->bat[1]);
    draw_battery_box(580, 358, 210, 86, 2, &pack->bat[2]);

    /* average SOC
    uint8_t avg = (pack->bat[0].soc + pack->bat[1].soc) / 2U;
    EVE_color_rgb_burst(0xFFFFFFUL);
    EVE_cmd_text_burst(580 + 8, 410, 25, EVE_OPT_CENTERY, "Average:");
    char avg_str[8];
    snprintf(avg_str, sizeof(avg_str), "%d%%", (int)avg);
    EVE_color_rgb_burst(soc_color(avg));
    EVE_cmd_text_burst(790 - 8, 410, 29,EVE_OPT_RIGHTX | EVE_OPT_CENTERY, avg_str); */
}

// ------ Alarms -----------------------------------------------------------------

static void draw_warnings(const battery_pack_t *pack, const sensor_data_t *sens)
{
    const int16_t  WX = 8;
    const int16_t  WY = 36;
    const uint16_t WW = 200;
    const uint16_t WH = 300;

    // outer box
    EVE_color_rgb_burst(0x111111UL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(WX * 16,          WY * 16);
    EVE_vertex2f_burst((WX + WW) * 16,   (WY + WH) * 16);
    EVE_end_burst();

    // title
    EVE_color_rgb_burst(0x888888UL);
    EVE_cmd_text_burst(WX + WW / 2, WY + 20, 28, EVE_OPT_CENTER, "Alarms");

    // line to separate the title
    EVE_color_rgb_burst(0xFFFFFFUL);
    EVE_line_width_burst(1 * 16);
    EVE_begin_burst(EVE_LINES);
    EVE_vertex2f_burst((WX + 8) * 16,          (WY + 34) * 16);
    EVE_vertex2f_burst((WX + WW - 8) * 16,     (WY + 34) * 16);
    EVE_end_burst();

    // variable for the three alarms
    bool  active[4];
    uint32_t color[4];
    char  title[4][12];
    char  detail[4][20];

    // TEMP B1
    active[0] = pack->bat[0].temp_c >= BAT_TEMP_WARN;
    color[0]  = active[0] ? 0xFF2200UL : 0x444444UL; // red or grey
    snprintf(title[0],  sizeof(title[0]),  "TEMP B1");
    snprintf(detail[0], sizeof(detail[0]), "%d C", (int)pack->bat[0].temp_c);

    // TEMP B2
    active[1] = pack->bat[1].temp_c >= BAT_TEMP_WARN;
    color[1]  = active[1] ? 0xFF2200UL : 0x444444UL;
    snprintf(title[1],  sizeof(title[1]),  "TEMP B2");
    snprintf(detail[1], sizeof(detail[1]), "%d C", (int)pack->bat[1].temp_c);

    // TEMP LVB1
    active[2] = pack->bat[2].temp_c >= BAT_TEMP_WARN;
    color[2]  = active[2] ? 0xFF2200UL : 0x444444UL;
    snprintf(title[2],  sizeof(title[2]),  "TEMP LVB1");
    snprintf(detail[2], sizeof(detail[2]), "%d C", (int)pack->bat[2].temp_c);

    // HUM
    active[3] = sens->humidity >= HUM_WARN;
    color[3]  = active[3] ? 0xFFAA00UL : 0x444444UL; // yellow or grey
    snprintf(title[3],  sizeof(title[3]),  "HUM");
    snprintf(detail[3], sizeof(detail[3]), "%d %%", (int)sens->humidity);

    // draw the 3 lines 
    int16_t wy = WY + 44;
    for (int i = 0; i < 4; i++) {

        // background line
        EVE_color_rgb_burst(active[i] ? 0x1a0a0aUL : 0x0f0f0fUL);
        EVE_begin_burst(EVE_RECTS);
        EVE_vertex2f_burst((WX + 8) * 16,          wy * 16);
        EVE_vertex2f_burst((WX + WW - 8) * 16,     (wy + 53) * 16);
        EVE_end_burst();

        // triangle
        EVE_color_rgb_burst(color[i]);
        EVE_line_width_burst(2 * 16);
        EVE_begin_burst(EVE_LINE_STRIP);
        EVE_vertex2f_burst((WX + 14) * 16,  (wy + 46) * 16);
        EVE_vertex2f_burst((WX + 26) * 16,  (wy + 18) * 16);
        EVE_vertex2f_burst((WX + 38) * 16,  (wy + 46) * 16);
        EVE_vertex2f_burst((WX + 14) * 16,  (wy + 46) * 16);
        EVE_end_burst();

        // '!' inside the triangle 
        if (active[i]) {
            EVE_color_rgb_burst(color[i]);
            EVE_cmd_text_burst(WX + 26, wy + 36, 22, EVE_OPT_CENTER, "!");
        }

        // title 
        EVE_color_rgb_burst(color[i]);
        EVE_cmd_text_burst(WX + 46, wy + 18, 27, 0, title[i]);

        // details 
        EVE_color_rgb_burst(active[i] ? 0xAAAAAAUL : 0x333333UL);
        EVE_cmd_text_burst(WX + 46, wy + 38, 26, 0, detail[i]);

        wy += 64;  // 56 high + 8 gap
    }
}

// ------ MQTT messages ------------------------------------------------------

static void draw_mqtt_bar(const mqtt_messages_t *msgs)
{
    const int16_t  MX = 0;
    const int16_t  MY = 342;
    const uint16_t MW = 570;
    const uint16_t MH = 148;

    EVE_color_rgb_burst(0x0d0d0dUL);
    EVE_begin_burst(EVE_RECTS);
    EVE_vertex2f_burst(MX * 16,        MY * 16);
    EVE_vertex2f_burst((MX+MW) * 16,   (MY+MH) * 16);
    EVE_end_burst();

    EVE_color_rgb_burst(0x444444UL);

    for (int i = 0; i < msgs->count && i < MQTT_MSG_MAX_ROWS; i++) {
        if (msgs->lines[i][0] == '\0') continue;

        uint32_t col = (i == 0) ? 0xCCCCCCUL : 0x555555UL;
        EVE_color_rgb_burst(col);
        EVE_cmd_text_burst(MX + 10,MY + 28 + i * 42,25, EVE_OPT_CENTERY,msgs->lines[i]);
    }
}
// ------ Fake MQTT ----------
static void fake_mqtt_task(void *pvParameters)
{
    // range value that change every 10 sec 
    const float ranges[][2] = {
        {10.0f, 25.0f},
        {20.0f, 35.0f},
        {30.0f, 45.0f},
        {15.0f, 40.0f},
    };
    const char *messages[] = {
        "Good job!!",
        "Keep power steady",
        "Last Lap Time: 22.01s",
        "good, keep going!",
    };

    int idx = 0;
    const int n = 4;

    while (1) {
        // write the value on the queue
        power_window_t pw = {
            .range_min_kw = ranges[idx][0],
            .range_max_kw = ranges[idx][1],
            .active       = true
        };
        xQueueOverwrite(q_power_window, &pw);
        ESP_LOGI(LOG_TAG, "MQTT power window: %.0f -> %.0f kW",(double)pw.range_min_kw, (double)pw.range_max_kw);

        // write new messages 
        mqtt_messages_t msgs;

        // check the status of the queue
        if (xQueueReceive(q_mqtt_msgs, &msgs, 0) != pdTRUE) {
            memset(&msgs, 0, sizeof(msgs));
        }

        // shift the message of one position 
        for (int i = MQTT_MSG_MAX_ROWS - 1; i > 0; i--) {
            strncpy(msgs.lines[i], msgs.lines[i - 1],MQTT_MSG_MAX_LEN - 1);
            msgs.lines[i][MQTT_MSG_MAX_LEN - 1] = '\0';
        }

        // new message on top of the queue 
        strncpy(msgs.lines[0], messages[idx], MQTT_MSG_MAX_LEN - 1);
        msgs.lines[0][MQTT_MSG_MAX_LEN - 1] = '\0';
        msgs.count = MQTT_MSG_MAX_ROWS;

        ESP_LOGI("MQTT", "count=%d", msgs.count);
        ESP_LOGI("MQTT", "lines[0]=%s", msgs.lines[0]);
        ESP_LOGI("MQTT", "lines[1]=%s", msgs.lines[1]);
        ESP_LOGI("MQTT", "lines[2]=%s", msgs.lines[2]);
        xQueueOverwrite(q_mqtt_msgs, &msgs);
        ESP_LOGI(LOG_TAG, "MQTT msg: %s", messages[idx]);

        idx = (idx + 1) % n;

        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 secondi
    }
}


// ------ Fake Can Task -------------------------------------------

static void fake_can_task(void *pvParameters)
{
    motor_data_t   motor  = {0};
    battery_pack_t pack   = {0};
    sensor_data_t  sensor = {0};

    pack.bat[0].soc     = 85U;
    pack.bat[0].voltage = 48.1f;
    pack.bat[0].current = 8.2f;
    pack.bat[0].temp_c  = 38U;

    pack.bat[1].soc     = 71U;
    pack.bat[1].voltage = 47.8f;
    pack.bat[1].current = 8.4f;
    pack.bat[1].temp_c  = 62U;

    pack.bat[2].soc     = 71U;
    pack.bat[2].voltage = 12.1f;
    pack.bat[2].current = 8.4f;
    pack.bat[2].temp_c  = 62U;

    sensor.humidity = 82U;

    int dir = 1;

    while (1) {
        motor.rpm      += dir * 30;
        motor.speed_kn  = motor.rpm / 100.0f;
        motor.power_kw  = motor.rpm / 50.0f;

        if (motor.rpm >= MAX_RPM) dir = -1;
        if (motor.rpm == 0)       dir =  1;

        xQueueOverwrite(q_motor,   &motor);
        xQueueOverwrite(q_battery, &pack);
        xQueueOverwrite(q_sensor,  &sensor);

        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

// -------- Button Task ---------------------------------------------------
typedef struct {
    uint8_t  cruise_on;     // 1 = attiva cruise, 0 = disattiva
    int16_t  cruise_rpm;    // rpm fissi (signed)
} __attribute__((packed)) cruise_cmd_t;

static void send_cruise_cmd(bool on, int16_t rpm)
{
    cruise_cmd_t c = { .cruise_on = on ? 1 : 0, .cruise_rpm = rpm };

    twai_message_t frame = {
        .identifier       = CRUISE_CAN_ID,
        .extd             = 0,                    
        .data_length_code = sizeof(cruise_cmd_t),
    };
    memcpy(frame.data, &c, sizeof(c));

    esp_err_t err = twai_transmit(&frame, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(LOG_TAG, "Cruise cmd sent: on=%d rpm=%d", c.cruise_on, c.cruise_rpm);
    } else {
        ESP_LOGE(LOG_TAG, "Cruise cmd TX failed: %s", esp_err_to_name(err));
    }
}

static void button_task(void *pvParameters)
{
    bool btn_prev    = true;   // bottone schermo
    bool cruise_prev = true;   // bottone cruise

    while (1) {
        bool btn    = gpio_get_level(BTN_PIN);
        bool cruise = gpio_get_level(CRUISE_BTN_PIN);

        // bottone schermo (già esistente)
        if (btn_prev == true && btn == false) {
            current_screen = (current_screen + 1) % 3;
        }

        // bottone cruise: alla premuta, manda il frame
        if (cruise_prev == true && cruise == false) {
            send_cruise_cmd(true, CRUISE_RPM);
        }

        btn_prev    = btn;
        cruise_prev = cruise;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------CAN Receiver Task ---------------------------------------------
void can_receiver_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    twai_message_t rxMessage;
    while (1) {
        //ESP_LOGI("CAN", "while 1");
        while (twai_receive(&rxMessage, pdMS_TO_TICKS(10)) == ESP_OK) {
            //ESP_LOGI("CAN", "while twai receive");
            switch (rxMessage.identifier) {

                case GVI_MOTOR_GVI_STATUS_TX_PDO1_FRAME_ID: {  // 0x186
                    struct gvi_motor_gvi_status_tx_pdo1_t m;
                    if (gvi_motor_gvi_status_tx_pdo1_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        
                        motor_data_t motor;
                        if (xQueuePeek(q_motor, &motor, 0) != pdTRUE) {
                            memset(&motor, 0, sizeof(motor));
                        }

                        motor.rpm = (uint16_t)gvi_motor_gvi_status_tx_pdo1_rotor_speed_in_rpm_decode(m.rotor_speed_in_rpm);
                        motor.voltagedc = (int16_t)gvi_motor_gvi_status_tx_pdo1_signal_f_iltered_voltage_decode(m.signal_f_iltered_voltage);

                        xQueueOverwrite(q_motor, &motor);
                    }
                    break;
                }

                case GVI_MOTOR_DIAGNOSTIC1_TX_PDO2_FRAME_ID: {  // 0x286
                    struct gvi_motor_diagnostic1_tx_pdo2_t m;
                    if (gvi_motor_diagnostic1_tx_pdo2_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {
                        
                        motor_data_t motor;
                        if (xQueuePeek(q_motor, &motor, 0) != pdTRUE) {
                            memset(&motor, 0, sizeof(motor));
                        }

                        motor.currentdc = (int16_t)gvi_motor_diagnostic1_tx_pdo2_dc_bus_current_decode(m.dc_bus_current);

                        xQueueOverwrite(q_motor, &motor);
                    }
                    break;
                }

                case GVI_MOTOR_DIAGNOSTIC3_TX_PDO4_FRAME_ID: {  // 0x486
                    struct gvi_motor_diagnostic3_tx_pdo4_t m;
                    if (gvi_motor_diagnostic3_tx_pdo4_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0) {

                        motor_data_t motor;
                        if (xQueuePeek(q_motor, &motor, 0) != pdTRUE) {
                            memset(&motor, 0, sizeof(motor));
                        }

                        motor.temp = m.motor_temp;

                        xQueueOverwrite(q_motor, &motor);
                    }
                    break;
                }

                case 0x07: {  // GPS Speed
                    struct batteries_gps_motor_2_gnss_speed_t msg;
                    batteries_gps_motor_2_gnss_speed_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                    motor_data_t motor;
                    if (xQueuePeek(q_motor, &motor, 0) != pdTRUE) {
                        memset(&motor, 0, sizeof(motor));
                    }
                    motor.speed_kn = batteries_gps_motor_2_gnss_speed_speed_decode(msg.speed);
                    xQueueOverwrite(q_motor, &motor);
                    break;
                }

                case 0x04028001: {  // Battery_VoltCurrSOC_1
                    struct batteries_total_info_0_batt1_t m;
                    if (batteries_total_info_0_batt1_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[0].voltage = batteries_total_info_0_batt1_voltage_1_decode(m.voltage_1);
                        pack.bat[0].current = batteries_total_info_0_batt1_current_1_decode(m.current_1);
                        pack.bat[0].soc = (uint8_t)(batteries_total_info_0_batt1_soc_1_decode(m.soc_1));
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;  
                }

                case 0x04028002: {  // Battery_VoltCurrSOC_2
                    struct batteries_total_info_0_batt2_t m;
                    ESP_LOGI("CAN", "dentro 04028002 unpack");
                    if (batteries_total_info_0_batt2_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        ESP_LOGI("CAN", "dentro 04028002 unpack");
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[1].voltage = batteries_total_info_0_batt2_voltage_2_decode(m.voltage_2);
                        pack.bat[1].current = batteries_total_info_0_batt2_current_2_decode(m.current_2);
                        pack.bat[1].soc = (uint8_t)(batteries_total_info_0_batt2_soc_2_decode(m.soc_2));
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;  
                }

                case 0x04058001: {  // temperatures b1
                    struct batteries_temperature_1_t m;
                    if (batteries_temperature_1_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[0].temp_c = batteries_temperature_1_max_t_1_decode(m.max_t_1);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x04058002: {  // temperatures b2
                    struct batteries_temperature_2_t m;
                    if (batteries_temperature_2_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[1].temp_c = batteries_temperature_2_max_t_2_decode(m.max_t_2);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    //ESP_LOGI("CAN", "ciao funziona");
                    break;
                }

                case 0x020: {  // DHT humidity
                    struct dht_lv_batt_dht_lv_batt_t msg;
                    dht_lv_batt_dht_lv_batt_unpack(&msg,rxMessage.data, rxMessage.data_length_code);

                    sensor_data_t sensor;
                    battery_pack_t pack;
                    if (xQueuePeek(q_sensor, &sensor, 0) != pdTRUE) {
                        memset(&sensor, 0, sizeof(sensor));
                    }

                    pack.bat[2].temp_c = dht_lv_batt_dht_lv_batt_temp_dht_decode(msg.temp_dht);
                    sensor.humidity = (uint8_t)dht_lv_batt_dht_lv_batt_hum_dht_decode(msg.hum_dht);
                    xQueueOverwrite(q_sensor, &sensor);
                    break;
                }



                default:
                //ESP_LOGI("CAN", "id=0x%08lX extd=%d dlc=%d",
             //(unsigned long)rxMessage.identifier,
             //rxMessage.extd, rxMessage.data_length_code);
                    break;
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
}

// ------ Display Task ----------------------------------------------------

static void eve_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    EVE_init_spi();

    uint8_t ret = EVE_init();
    if (ret != 0U) {
        ESP_LOGE(LOG_TAG, "EVE_init fallito ret=%d", ret);
        esp_task_wdt_delete(NULL);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(LOG_TAG, "FT812 ok");

    EVE_memWrite16(REG_PWM_HZ,   1000U);
    EVE_memWrite8(REG_PWM_DUTY,  96U);

    motor_data_t    motor    = {0};
    battery_pack_t  pack     = {0};
    sensor_data_t   sensor   = {0};
    power_window_t  pw       = {0};
    mqtt_messages_t msgs     = {0};

    // initial message
    strncpy(msgs.lines[0], "",
            MQTT_MSG_MAX_LEN - 1);
    msgs.count = MQTT_MSG_MAX_ROWS;

    while (1) {
        xQueuePeek(q_motor,         &motor,  0);
        xQueuePeek(q_battery,       &pack,   0);
        xQueuePeek(q_sensor,        &sensor, 0);
        xQueuePeek(q_power_window,  &pw,     0);
        motor.power_kw = motor.currentdc * motor.voltagedc / 1000.0f;

        mqtt_messages_t new_msgs;
        if (xQueuePeek(q_mqtt_msgs, &new_msgs, 0) == pdTRUE) {
            msgs = new_msgs;
        }
        //int64_t t0 = esp_timer_get_time();

        EVE_start_cmd_burst();
        EVE_cmd_dlstart_burst();
        EVE_clear_color_rgb_burst(0x0a0a0aUL);
        EVE_clear_burst(1U, 1U, 1U);
        draw_warnings(&pack, &sensor);
        switch (current_screen) {
            case 0:
                draw_rpm_bar(motor.rpm, MAX_RPM, "rpm");
                draw_gauge_generic(motor.power_kw, MAX_POWER_KW, "kW", &pw);
                draw_speed(motor.speed_kn, "Speed","knots");
                break;
            case 1:
                draw_rpm_bar(motor.rpm, MAX_RPM, "rpm");
                draw_gauge_generic(motor.speed_kn, MAX_SPD, "knots", NULL);
                draw_speed(motor.power_kw, "Power","kW");
                break;
            case 2:
                draw_rpm_bar(motor.power_kw, MAX_POWER_KW, "kW");
                draw_gauge_generic(motor.rpm, MAX_RPM, "rpm", NULL);
                draw_speed(motor.speed_kn, "Speed","knots");
                break;
        }
        draw_batteries(&pack);
        draw_mqtt_bar(&msgs);
        draw_connectivity_status(wifi_connected, mqtt_connected);

        EVE_display_burst();
        EVE_cmd_swap_burst();
        EVE_end_cmd_burst();
        EVE_execute_cmd();
        //int64_t t1 = esp_timer_get_time();
        //ESP_LOGI(LOG_TAG, "frame: %lld us", t1 - t0);
        esp_task_wdt_reset();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



// ------ MAIN ------------------------------------------

void app_main(void)
{
    // button configurations
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_PIN) | (1ULL << CRUISE_BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg);

    // queues initilization
    q_motor         = xQueueCreate(1, sizeof(motor_data_t));
    q_battery       = xQueueCreate(1, sizeof(battery_pack_t));
    q_sensor        = xQueueCreate(1, sizeof(sensor_data_t));
    q_power_window  = xQueueCreate(1, sizeof(power_window_t));
    q_mqtt_msgs     = xQueueCreate(1, sizeof(mqtt_messages_t));

    // initial values in the queue
    motor_data_t   m = {0};   xQueueSend(q_motor,        &m, 0);
    battery_pack_t b = {0};   xQueueSend(q_battery,      &b, 0);
    sensor_data_t  s = {0};   xQueueSend(q_sensor,       &s, 0);
    power_window_t p = {0};   xQueueSend(q_power_window, &p, 0);


    // wifi and mqtt initialization
    wifi_init_sta(); // connection to wifi
    mqtt_app_start(); // mqtt connection

    ESP_LOGI("TWAI", "Initialazing TWAI in NORMAL mode...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_LISTEN_ONLY); 
    g_config.tx_queue_len = 20;
    g_config.rx_queue_len = 30;
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  
 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_OK)
    {
        ESP_LOGI("TWAI", "TWAI installed successfully!");
        if (twai_start() == ESP_OK)
        {
            ESP_LOGI("TWAI", "TWAI started in normal mode!");

        }else{
            ESP_LOGE("TWAI","Failed to start TWAI");
            return;
        }
        
    }else{
        ESP_LOGE("TWAI","Failed to install TWAI");
        return;

    }

    //xTaskCreate(fake_can_task, "can",     3072, NULL, 5, NULL);
    //xTaskCreate(fake_mqtt_task, "fake_mqtt", 4096, NULL, 3, NULL);
    xTaskCreatePinnedToCore(eve_task, "display", 6144, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(can_receiver_task, "can_task", 6144, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "button", 4096, NULL, 3, NULL, 0);
}

/*case 0x359: {  // Battery_VoltCurrTemp_1
                    struct batteries_gps_motor_2_battery_volt_curr_temp_1_t m;
                    if (batteries_gps_motor_2_battery_volt_curr_temp_1_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[0].voltage = batteries_gps_motor_2_battery_volt_curr_temp_1_battery_voltage_1_decode(m.battery_voltage_1);
                        pack.bat[0].current = batteries_gps_motor_2_battery_volt_curr_temp_1_battery_current_1_decode(m.battery_current_1);
                        pack.bat[0].temp_c  = (uint8_t)batteries_gps_motor_2_battery_volt_curr_temp_1_battery_temp_1_decode(m.battery_temp_1);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x360: {  // Battery_VoltCurrTemp_2
                    struct batteries_gps_motor_2_battery_volt_curr_temp_2_t m;
                    if (batteries_gps_motor_2_battery_volt_curr_temp_2_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[1].voltage = batteries_gps_motor_2_battery_volt_curr_temp_2_battery_voltage_2_decode(m.battery_voltage_2);
                        pack.bat[1].current = batteries_gps_motor_2_battery_volt_curr_temp_2_battery_current_2_decode(m.battery_current_2);
                        pack.bat[1].temp_c  = (uint8_t)batteries_gps_motor_2_battery_volt_curr_temp_2_battery_temp_2_decode(m.battery_temp_2);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x361: {  // Battery_VoltCurrTemp_3
                    struct batteries_gps_motor_2_battery_volt_curr_temp_3_t m;
                    if (batteries_gps_motor_2_battery_volt_curr_temp_3_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[2].voltage = batteries_gps_motor_2_battery_volt_curr_temp_3_battery_voltage_3_decode(m.battery_voltage_3);
                        pack.bat[2].current = batteries_gps_motor_2_battery_volt_curr_temp_3_battery_current_3_decode(m.battery_current_3);
                        pack.bat[2].temp_c  = (uint8_t)batteries_gps_motor_2_battery_volt_curr_temp_3_battery_temp_3_decode(m.battery_temp_3);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x362: {  // Battery_VoltCurrTemp_4
                    struct batteries_gps_motor_2_battery_volt_curr_temp_4_t m;
                    if (batteries_gps_motor_2_battery_volt_curr_temp_4_unpack(&m, rxMessage.data, rxMessage.data_length_code) == 0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[3].voltage = batteries_gps_motor_2_battery_volt_curr_temp_4_battery_voltage_4_decode(m.battery_voltage_4);
                        pack.bat[3].current = batteries_gps_motor_2_battery_volt_curr_temp_4_battery_current_4_decode(m.battery_current_4);
                        pack.bat[3].temp_c  = (uint8_t)batteries_gps_motor_2_battery_volt_curr_temp_4_battery_temp_4_decode(m.battery_temp_4);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x355: {  // Battery 1 SOC
                    struct batteries_gps_motor_2_soc_soh_1_t msg;
                    if(batteries_gps_motor_2_soc_soh_1_unpack(&msg,rxMessage.data, rxMessage.data_length_code)==0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[0].soc = (uint8_t)(msg.high_res_soc_1 * 0.01f);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x356: {  // Battery 2 SOC
                    struct batteries_gps_motor_2_soc_soh_2_t msg;
                    if(batteries_gps_motor_2_soc_soh_2_unpack(&msg,rxMessage.data, rxMessage.data_length_code)==0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[1].soc = (uint8_t)(msg.high_res_soc_2 * 0.01f);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x357: {  // Battery 3 SOC
                    struct batteries_gps_motor_2_soc_soh_3_t msg;
                    if(batteries_gps_motor_2_soc_soh_3_unpack(&msg,rxMessage.data, rxMessage.data_length_code)==0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[2].soc = (uint8_t)(msg.high_res_soc_3 * 0.01f);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }

                case 0x358: {  // Battery 4 SOC
                    struct batteries_gps_motor_2_soc_soh_4_t msg;
                    if(batteries_gps_motor_2_soc_soh_4_unpack(&msg,rxMessage.data, rxMessage.data_length_code)==0){
                        battery_pack_t pack;
                        if (xQueuePeek(q_battery, &pack, 0) != pdTRUE) {
                            memset(&pack, 0, sizeof(pack));
                        }
                        pack.bat[3].soc = (uint8_t)(msg.high_res_soc_4 * 0.01f);
                        xQueueOverwrite(q_battery, &pack);
                    }
                    break;
                }*/