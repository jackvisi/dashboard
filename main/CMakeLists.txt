idf_component_register(
    SRCS "dashboard.c" "batteries_gps_motor_2.h" "nma2000_motor.h" "motor_estimated.h"
    INCLUDE_DIRS "."
    REQUIRES EVE esp_timer nvs_flash esp_wifi esp_event esp_netif mqtt json
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    EVE_NHD_70
    FT812
)