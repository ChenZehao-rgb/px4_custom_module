set(config_module_list_external
    modules/tele1_sensor
    PARENT_SCOPE
)

add_subdirectory(msg)
add_subdirectory(src/modules/tele1_sensor)