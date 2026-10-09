idf_component_register(
    SRCS
        "EVE_commands.c"
        "EVE_supplemental.c"
        "EVE_target.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_driver_spi
        esp_driver_gpio
        esp_timer
        freertos
        log 
        nvs_flash 
        esp_wifi 
        esp_event 
        esp_netif 
        mqtt
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EVE_NHD_70
    FT812

)

# ESP-IDF 6.x compatibility:
# EVE headers include task.h without including FreeRTOS.h first.
target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
    -include freertos/FreeRTOS.h
)
