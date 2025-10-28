// pwm_raw_control.cpp (PX4 v1.15+ 适配版：通过 actuator_test 发布)
// 构建：放到你的外置模块目录并加入 CMake 的 PX4_MODULE_OUTER_SRCS

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>                 // hrt_absolute_time
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>
#include <math.h>   // 或者 <cmath>


extern "C" __EXPORT int pwm_raw_control_main(int argc, char *argv[]);

using namespace time_literals;

class PwmRawControl : public ModuleBase<PwmRawControl>
{
public:
    PwmRawControl() = default;
    ~PwmRawControl() override = default;

    void run() override;
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int, char *[]) { return print_usage("Unknown command"); }
    static PwmRawControl *instantiate(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

private:
    static constexpr unsigned kMax = 8;

    // 目标功能枚举（Motor1..）
    uint16_t _func[kMax] {};
    unsigned _num{0};

    // 以微秒输入，再映射到 [0,1]
    uint16_t _pwm_us{1500};
    uint16_t _pwm_min{1000};
    uint16_t _pwm_max{2000};

    // 发布器
    uORB::Publication<actuator_test_s> _pub{ORB_ID(actuator_test)};

    float pwm_to_norm_motor(uint16_t us) const {
        if (_pwm_max <= _pwm_min) return NAN;
        float x = (float(us) - float(_pwm_min)) / float(_pwm_max - _pwm_min);
        if (x < 0.f) x = 0.f;
        if (x > 1.f) x = 1.f;
        return x; // 电机用 [0,1]
    }

    void send_action(uint8_t action, float value_norm, uint16_t func, uint32_t timeout_ms) {
        actuator_test_s m{};
        m.timestamp = hrt_absolute_time();
        m.action    = action;
        m.function  = func;
        m.value     = value_norm;   // 电机：[0..1]；NaN=停止
        m.timeout_ms= timeout_ms;   // 0=不超时，持续控制
        _pub.publish(m);
    }
};

void PwmRawControl::run()
{
    const float v = pwm_to_norm_motor(_pwm_us);

    PX4_INFO("actuator_test: %u motor(s) -> %u us (norm=%.3f), min=%u max=%u",
         _num,
         static_cast<unsigned>(_pwm_us),
         static_cast<double>(v),
         static_cast<unsigned>(_pwm_min),
         static_cast<unsigned>(_pwm_max));


    // 进入控制：对所选 Motor 功能位下发 DO_CONTROL
    for (unsigned i = 0; i < _num; i++) {
        send_action(actuator_test_s::ACTION_DO_CONTROL, v, _func[i], 0 /*no timeout*/);
    }

    // 周期性刷新，避免看门狗/其他模块接管（~10 Hz）
    while (!should_exit()) {
        for (unsigned i = 0; i < _num; i++) {
            send_action(actuator_test_s::ACTION_DO_CONTROL, v, _func[i], 0);
        }
        px4_usleep(100_ms);
    }

    // 退出时释放控制
    for (unsigned i = 0; i < _num; i++) {
        send_action(actuator_test_s::ACTION_RELEASE_CONTROL, NAN, _func[i], 0);
    }

    PX4_INFO("released control, exiting");
}

int PwmRawControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("pwm_raw_control",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2000,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);
    if (_task_id < 0) {
        PX4_ERR("task start failed (%d)", errno);
        return -errno;
    }
    return 0;
}

PwmRawControl *PwmRawControl::instantiate(int argc, char *argv[])
{
    auto *inst = new PwmRawControl();
    if (!inst) { PX4_ERR("alloc failed"); return nullptr; }

    int ch;
    const char *chan = nullptr;

    int myoptind = 1;
    const char *myoptarg = nullptr;

    // 5参版 px4_getopt：argc, argv, "opts", &myoptind, &myoptarg
    while ((ch = px4_getopt(argc, argv, "c:p:m:x:", &myoptind, &myoptarg)) != -1) {
        switch (ch) {
        case 'c':
            chan = myoptarg;                           // 例如 "1234"
            break;
        case 'p':
            inst->_pwm_us = static_cast<uint16_t>(atoi(myoptarg));
            break;
        case 'm':
            inst->_pwm_min = static_cast<uint16_t>(atoi(myoptarg));
            break;
        case 'x':
            inst->_pwm_max = static_cast<uint16_t>(atoi(myoptarg));
            break;
        default:
            delete inst;
            return nullptr;
        }
    }

    // 把 "1..8" 映射到 FUNCTION_MOTOR1..8
    if (chan) {
        for (const char *p = chan; *p && inst->_num < kMax; ++p) {
            if (*p >= '1' && *p <= '8') {
                unsigned idx = static_cast<unsigned>(*p - '1');
                inst->_func[inst->_num++] =
                    static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + idx);
            }
        }
    } else {
        for (unsigned i = 0; i < 4; i++) {
            inst->_func[inst->_num++] =
                static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + i);
        }
    }

    return inst;
}


int PwmRawControl::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION(
        R"(
### 概述
通过 uORB `actuator_test` 旁路控制电机输出（仅在未解锁时有效）。
承接旧版 `pwm`/ioctl 风格：你仍然用微秒 `-p` 表达目标，我们内部映射到 [0,1]。

### 用法
pwm_raw_control start [-c channels] [-p pwm_usec] [-m min] [-x max]
  -c  选择电机通道（数字 '1'..'8'，如 1234），默认 1..4
  -p  目标 PWM 脉宽（微秒，默认 1500）
  -m  PWM 最小值（默认 1000）
  -x  PWM 最大值（默认 2000）

停止与状态命令使用 ModuleBase 默认命令（stop/status）。
)");
    PRINT_MODULE_USAGE_NAME("pwm_raw_control", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, nullptr, "通道列表 e.g. 1234", false);
    PRINT_MODULE_USAGE_PARAM_INT('p', 1500, 800, 2200, "PWM(us)", false);
    PRINT_MODULE_USAGE_PARAM_INT('m', 1000, 500, 1700, "PWM 最小", true);
    PRINT_MODULE_USAGE_PARAM_INT('x', 2000, 1800, 2500, "PWM 最大", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int pwm_raw_control_main(int argc, char *argv[])
{
    return PwmRawControl::main(argc, argv);
}
