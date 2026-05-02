idf_component_register(
    SRCS
        "EVE_commands.c"
        "EVE_supplemental.c"
        "EVE_target.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
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