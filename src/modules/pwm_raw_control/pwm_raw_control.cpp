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
#include <float.h>
#include <vector>
#include <errno.h>

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

    // 自动模式开关
    bool _mode_sweep{false};
    bool _mode_dither{false};

    // --- Sweep 参数 ---
    float    _sweep_lo_pct{5.f};      // 起始百分比（含）
    float    _sweep_hi_pct{35.f};     // 结束百分比（含）
    float    _sweep_step_pct{5.f};    // 台阶步长
    uint32_t _sweep_hold_ms{20000};   // 每个台阶保持时长
    uint32_t _sweep_gap_ms{0};        // 台阶之间的空档，若>0则在空档内释放控制
    unsigned _sweep_repeats{1};       // 完整“上去再下来”为1轮

    // --- Dither 参数 ---
    float    _dither_base_pct{20.f};  // 基准百分比
    float    _dither_amp_pct{5.f};    // 上下幅度（±）
    uint32_t _dither_hold_ms{20000};  // 高/低各保持时长
    unsigned _dither_cycles{10};      // 高低为一对，循环次数

    // 发布器
    uORB::Publication<actuator_test_s> _pub{ORB_ID(actuator_test)};

    static inline float clamp01(float x) {
        if (x < 0.f) return 0.f;
        if (x > 1.f) return 1.f;
        return x;
    }

    static inline float clamp100(float x) {
        if (x < 0.f) return 0.f;
        if (x > 100.f) return 100.f;
        return x;
    }
    // 将 us → [0,1]（电机）
    float pwm_to_norm_motor(uint16_t us) const {
        if (_pwm_max <= _pwm_min) return NAN;
        float x = (float(us) - float(_pwm_min)) / float(_pwm_max - _pwm_min);
        return clamp01(x);
    }

    // 将 百分比[0,100] → [0,1]（电机）
    float pct_to_norm_motor(float pct) const {
        float p = clamp100(pct) / 100.f;
        // 线性映射等价于 us = min + p*(max-min)，但 actuator_test 需要 [0,1]
        return p;
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

    // 对所有选中电机统一发送一次
    void send_all(uint8_t action, float value_norm, uint32_t timeout_ms) {
        for (unsigned i = 0; i < _num; i++) {
            send_action(action, value_norm, _func[i], timeout_ms);
        }
    }

    // 在一段时间内“保持”某个目标（周期刷新以防其他模块接管）
    void hold_value_for_ms(float value_norm, uint32_t hold_ms) {
        const uint64_t t_end = hrt_absolute_time() + (uint64_t)hold_ms * 1000ULL;
        while (!should_exit() && hrt_absolute_time() < t_end) {
            send_all(actuator_test_s::ACTION_DO_CONTROL, value_norm, 0);
            px4_usleep(100_ms); // 10 Hz 保活
        }
    }

    // 台阶之间可选“空档”
    void do_gap_ms(uint32_t gap_ms) {
        if (gap_ms == 0) return;
        send_all(actuator_test_s::ACTION_RELEASE_CONTROL, NAN, 0);
        const uint64_t t_end = hrt_absolute_time() + (uint64_t)gap_ms * 1000ULL;
        while (!should_exit() && hrt_absolute_time() < t_end) {
            px4_usleep(100_ms);
        }
    }

    void run_manual();
    void run_sweep();
    void run_dither();
};

void PwmRawControl::run_manual()
{
    const float v = pwm_to_norm_motor(_pwm_us);

    PX4_INFO("Manual: %u motor(s) -> %u us (norm=%.3f), min=%u max=%u",
             _num,
             static_cast<unsigned>(_pwm_us),
             static_cast<double>(v),
             static_cast<unsigned>(_pwm_min),
             static_cast<unsigned>(_pwm_max));

    // 进入控制
    send_all(actuator_test_s::ACTION_DO_CONTROL, v, 0);

    // 周期性刷新
    while (!should_exit()) {
        send_all(actuator_test_s::ACTION_DO_CONTROL, v, 0);
        px4_usleep(100_ms);
    }

    // 退出释放
    send_all(actuator_test_s::ACTION_RELEASE_CONTROL, NAN, 0);
    PX4_INFO("released control, exiting");
}

void PwmRawControl::run_sweep()
{
    // 构造一次完整序列：lo..hi..lo（hi 会自然重复一次）
    std::vector<float> seq;
    for (float p = _sweep_lo_pct; p <= _sweep_hi_pct + FLT_EPSILON; p += _sweep_step_pct) {
        seq.push_back(p);
    }
    for (float p = _sweep_hi_pct; p >= _sweep_lo_pct - FLT_EPSILON; p -= _sweep_step_pct) {
        seq.push_back(p);
    }

    PX4_INFO("Sweep mode: %u motor(s), lo=%.1f%% hi=%.1f%% step=%.1f%% hold=%ums gap=%ums repeats=%u",
             _num, (double)_sweep_lo_pct, (double)_sweep_hi_pct, (double)_sweep_step_pct,
             _sweep_hold_ms, _sweep_gap_ms, _sweep_repeats);

    // 开始循环
    for (unsigned r = 0; r < _sweep_repeats && !should_exit(); ++r) {
        for (size_t i = 0; i < seq.size() && !should_exit(); ++i) {
            float v = pct_to_norm_motor(seq[i]);
            PX4_INFO("Sweep[%u] step %zu / %zu : %.1f%% (norm=%.3f)",
                     r + 1, i + 1, seq.size(), (double)seq[i], (double)v);
            hold_value_for_ms(v, _sweep_hold_ms);
            do_gap_ms(_sweep_gap_ms);
        }
    }

    // 退出释放
    send_all(actuator_test_s::ACTION_RELEASE_CONTROL, NAN, 0);
    PX4_INFO("sweep done, released control, exiting");
}

void PwmRawControl::run_dither()
{
    // 计算高/低两个档位
    const float hi_pct = clamp100(_dither_base_pct + _dither_amp_pct);
    const float lo_pct = clamp100(_dither_base_pct - _dither_amp_pct);
    const float v_hi   = pct_to_norm_motor(hi_pct);
    const float v_lo   = pct_to_norm_motor(lo_pct);

    PX4_INFO("Dither mode: %u motor(s), base=%.1f%% amp=%.1f%% → [lo=%.1f%%, hi=%.1f%%], hold=%ums, cycles=%u",
             _num, (double)_dither_base_pct, (double)_dither_amp_pct,
             (double)lo_pct, (double)hi_pct, _dither_hold_ms, _dither_cycles);

    for (unsigned i = 0; i < _dither_cycles && !should_exit(); ++i) {
        PX4_INFO("Dither cycle %u / %u : HI", i + 1, _dither_cycles);
        hold_value_for_ms(v_hi, _dither_hold_ms);
        if (should_exit()) break;
        PX4_INFO("Dither cycle %u / %u : LO", i + 1, _dither_cycles);
        hold_value_for_ms(v_lo, _dither_hold_ms);
    }

    // 退出释放
    send_all(actuator_test_s::ACTION_RELEASE_CONTROL, NAN, 0);
    PX4_INFO("dither done, released control, exiting");
}

void PwmRawControl::run()
{
    // 统一在进入主循环前宣告 min/max
    PX4_INFO("PWM window: min=%u, max=%u", (unsigned)_pwm_min, (unsigned)_pwm_max);

    if (_mode_sweep) {
        run_sweep();
        return;
    }

    if (_mode_dither) {
        run_dither();
        return;
    }

    // 默认手动模式
    run_manual();
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

    // 选项说明：
    // -c 选择通道（如 1234）
    // -p 目标 PWM(us)（手动模式）
    // -m 最小 PWM(us)
    // -x 最大 PWM(us)
    //
    // 自动模式（2选1）：
    //   Sweep：
    //     -S  开启 Sweep
    //     -l  lo 百分比（默认 5）
    //     -u  hi 百分比（默认 35）
    //     -e  step 百分比（默认 5）
    //     -H  每台阶保持时长 ms（默认 20000）
    //     -G  台阶间空档 ms（默认 0；>0 时空档内会 RELEASE_CONTROL）
    //     -R  重复轮数（默认 1）
    //   Dither：
    //     -D  开启 Dither
    //     -b  基准百分比（默认 20）
    //     -a  幅度百分比（默认 5，表示 ±5）
    //     -h  每档保持时长 ms（默认 20000）
    //     -N  循环次数（默认 10）
    while ((ch = px4_getopt(argc, argv,
                            "c:p:m:x:Sl:u:e:H:G:R:D b:a:h:N:",
                            &myoptind, &myoptarg)) != -1) {
        switch (ch) {
        case 'c': chan = myoptarg; break;
        case 'p': inst->_pwm_us  = static_cast<uint16_t>(atoi(myoptarg)); break;
        case 'm': inst->_pwm_min = static_cast<uint16_t>(atoi(myoptarg)); break;
        case 'x': inst->_pwm_max = static_cast<uint16_t>(atoi(myoptarg)); break;

        case 'S': inst->_mode_sweep = true; break;
        case 'l': inst->_sweep_lo_pct   = atof(myoptarg); break;
        case 'u': inst->_sweep_hi_pct   = atof(myoptarg); break;
        case 'e': inst->_sweep_step_pct = atof(myoptarg); break;
        case 'H': inst->_sweep_hold_ms  = (uint32_t)atoi(myoptarg); break;
        case 'G': inst->_sweep_gap_ms   = (uint32_t)atoi(myoptarg); break;
        case 'R': inst->_sweep_repeats  = (unsigned)atoi(myoptarg); break;

        case 'D': inst->_mode_dither = true; break;
        case 'b': inst->_dither_base_pct = atof(myoptarg); break;
        case 'a': inst->_dither_amp_pct  = atof(myoptarg); break;
        case 'h': inst->_dither_hold_ms  = (uint32_t)atoi(myoptarg); break;
        case 'N': inst->_dither_cycles   = (unsigned)atoi(myoptarg); break;

        default:
            delete inst;
            return nullptr;
        }
    }

    // 互斥处理：若两个模式都被开启，则以 Sweep 优先（也可改为报错）
    if (inst->_mode_sweep && inst->_mode_dither) {
        PX4_WARN("Both -S and -D specified; using Sweep (-S).");
        inst->_mode_dither = false;
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

    // 参数合法化
    if (inst->_pwm_max <= inst->_pwm_min) {
        PX4_WARN("invalid min/max, correcting");
        std::swap(inst->_pwm_min, inst->_pwm_max);
    }
    inst->_sweep_lo_pct   = clamp100(inst->_sweep_lo_pct);
    inst->_sweep_hi_pct   = clamp100(inst->_sweep_hi_pct);
    inst->_sweep_step_pct = std::max(0.1f, inst->_sweep_step_pct);
    inst->_dither_base_pct= clamp100(inst->_dither_base_pct);
    inst->_dither_amp_pct = std::max(0.f, inst->_dither_amp_pct);

    return inst;
}

int PwmRawControl::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION(
        R"(
### 概述
通过 uORB `actuator_test` 旁路控制电机输出（未解锁时有效）。
保留手动微秒输入，同时新增两种自动测试模式：阶梯扫档（Sweep）与基准抖动（Dither）。
百分比基于 [-m, -x] 线性映射：0%→min，100%→max。

### 用法
# 手动（与旧版一致）
pwm_raw_control start [-c channels] [-p pwm_usec] [-m min] [-x max]

# 自动一：阶梯扫档（默认轨迹 5→10→…→35→35→30→…→5）
pwm_raw_control start -S [-c 1234] [-m 1000] [-x 2000] \
    [-l lo%] [-u hi%] [-e step%] [-H hold_ms] [-G gap_ms] [-R repeats]

# 自动二：基准抖动（围绕 base% ±amp%）
pwm_raw_control start -D [-c 1234] [-m 1000] [-x 2000] \
    [-b base%] [-a amp%] [-h hold_ms] [-N cycles]

### 参数
  -c  选择电机通道（数字 '1'..'8'，如 1234），默认 1..4
  -p  目标 PWM 脉宽（微秒，默认 1500）【仅手动】
  -m  PWM 最小值（默认 1000）
  -x  PWM 最大值（默认 2000）

【Sweep】
  -S  开启阶梯扫档模式
  -l  lo 百分比（默认 5）
  -u  hi 百分比（默认 35）
  -e  step 百分比（默认 5）
  -H  每台阶保持时长 ms（默认 20000）
  -G  台阶间空档 ms（默认 0；若>0，则空档期间会 RELEASE_CONTROL）
  -R  重复轮数（默认 1），每轮是“上去再下来”

【Dither】
  -D  开启基准抖动模式
  -b  基准百分比（默认 20）
  -a  幅度百分比（默认 5，表示 ±5）
  -h  每档保持时长 ms（默认 20000）
  -N  循环次数（默认 10），每次包含 HI 与 LO 各一次

停止与状态命令使用 ModuleBase 默认命令（stop/status）。
)");
    PRINT_MODULE_USAGE_NAME("pwm_raw_control", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, nullptr, "通道列表 e.g. 1234", false);
    PRINT_MODULE_USAGE_PARAM_INT('p', 1500, 800, 2200, "PWM(us)（手动模式）", true);
    PRINT_MODULE_USAGE_PARAM_INT('m', 1000, 500, 1700, "PWM 最小", true);
    PRINT_MODULE_USAGE_PARAM_INT('x', 2000, 1800, 2500, "PWM 最大", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("start -S", "阶梯扫档（指定 lo/hi/step/hold/gap/repeats）");
    PRINT_MODULE_USAGE_PARAM_FLOAT('l', 5, 0, 100, "lo 百分比", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('u', 35, 0, 100, "hi 百分比", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('e', 5, 0.1, 100, "step 百分比", true);
    PRINT_MODULE_USAGE_PARAM_INT('H', 20000, 0, 600000, "台阶保持 ms", true);
    PRINT_MODULE_USAGE_PARAM_INT('G', 0, 0, 600000, "台阶空档 ms（>0 时释放控制）", true);
    PRINT_MODULE_USAGE_PARAM_INT('R', 1, 1, 1000, "重复轮数", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("start -D", "基准抖动（指定 base/amp/hold/cycles）");
    PRINT_MODULE_USAGE_PARAM_FLOAT('b', 20, 0, 100, "基准百分比", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('a', 5, 0, 100, "幅度百分比（±）", true);
    PRINT_MODULE_USAGE_PARAM_INT('h', 20000, 1, 600000, "每档保持 ms", true);
    PRINT_MODULE_USAGE_PARAM_INT('N', 10, 1, 100000, "循环次数（HI+LO 为 1 次）", true);

    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int pwm_raw_control_main(int argc, char *argv[])
{
    return PwmRawControl::main(argc, argv);
}
