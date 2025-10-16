px4_add_module(
    MODULE modules__tele1_sensor
    MAIN tele1_sensor
    STACK_MAIN 2000
    SRCS
        tele1_sensor.cpp
    EXTERNAL
)