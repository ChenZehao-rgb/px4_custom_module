px4_add_module(
    MODULE modules__pwm_raw_control
    MAIN pwm_raw_control
    STACK_MAIN 2000
    SRCS
        pwm_raw_control.cpp
    EXTERNAL
)
