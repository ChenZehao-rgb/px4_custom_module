set(config_module_list_external
    modules/force_sensor
    modules/pwm_raw_control
    PARENT_SCOPE
)

add_subdirectory(msg)
add_subdirectory(src/modules/force_sensor)
add_subdirectory(src/modules/pwm_raw_control)
