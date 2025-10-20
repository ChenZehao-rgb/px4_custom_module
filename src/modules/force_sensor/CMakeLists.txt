px4_add_module(
    MODULE modules__force_sensor
    MAIN force_sensor
    STACK_MAIN 2000
    SRCS
        force_sensor.cpp
    EXTERNAL
)
