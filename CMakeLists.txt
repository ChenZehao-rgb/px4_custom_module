set(config_module_list_external
    modules/force_sensor
    PARENT_SCOPE
)

add_subdirectory(msg)
add_subdirectory(src/modules/force_sensor)
