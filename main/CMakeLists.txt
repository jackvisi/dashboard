idf_component_register(
    SRCS 
        "dashboard.c"
        "batteries_gps_motor_2.c"
        "nma2000_motor.c"
        "motor_estimated.c"
        "dht_lv_batt.c"
        "gvi_motor.c"
    INCLUDE_DIRS "."
    REQUIRES EVE esp_timer nvs_flash esp_wifi esp_event esp_netif mqtt json driver
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    EVE_NHD_70
    FT812
)