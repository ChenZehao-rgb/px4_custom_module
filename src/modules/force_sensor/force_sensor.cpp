/************************************************************************************
 * force_sensor.cpp (blocking reader for real device + ScheduledWorkItem for SIM)
 *
 * Features:
 *  - Real device: dedicated thread with poll()+read()  阻塞读，省 CPU、低延迟
 *  - SIM mode: ScheduledWorkItem 定时喂“R+16位数字”帧
 *  - 共用解析器 handle_bytes()，uORB 发布 force_sensor topic
 *
 * Usage:
 *   force_sensor start [-d <device>] [-b <baud>] [--sim [Hz]]
 *   force_sensor status
 *   force_sensor stop
 *
 ************************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/force_sensor.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>

using namespace time_literals;

class ForceSensor : public ModuleBase<ForceSensor>,
                    public ModuleParams,
                    public px4::ScheduledWorkItem
{
public:
    ForceSensor();
    ~ForceSensor() override;

    int init();

    static int task_spawn(int argc, char *argv[]);
    static ForceSensor *instantiate(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

private:
    /** 仿真 & 工作队列回调（仅 SIM 使用；实机模式不调度） */
    void Run() override;

    /** 串口配置（原始模式、8N1、无流控） */
    int configure_port(int fd, speed_t baud);

    /** int 波特率到 termios speed_t */
    static speed_t map_baud(int baudrate) {
        switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return B115200;
        }
    }

    /** 统一字节流解析：支持乱流/分段/粘包，17 字节成帧：'R' + 16 ASCII 数字 */
    void handle_bytes(const char *data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            char c = data[i];

            if (_frame_len == 0) {
                if (c == 'R') {
                    _frame_buf[_frame_len++] = c;
                }
                continue;
            }

            if (_frame_len < sizeof(_frame_buf)) {
                _frame_buf[_frame_len++] = c;
            }

            if (_frame_len == 17) {
                force_sensor_s msg{};
                msg.timestamp = hrt_absolute_time();

                for (int j = 0; j < 4; j++) {
                    char digits[5] = {0};
                    memcpy(digits, &_frame_buf[1 + j * 4], 4);
                    unsigned long val = strtoul(digits, nullptr, 10);
                    switch (j) {
                        case 0: msg.sensor1 = static_cast<uint16_t>(val); break;
                        case 1: msg.sensor2 = static_cast<uint16_t>(val); break;
                        case 2: msg.sensor3 = static_cast<uint16_t>(val); break;
                        case 3: msg.sensor4 = static_cast<uint16_t>(val); break;
                    }
                }

                _pub.publish(msg);
                _frame_len = 0; // 下一帧
            }
        }
    }

    /** 独立线程入口（trampoline） */
    static int reader_trampoline(int argc, char *argv[]);
    /** 阻塞读主循环（poll + read） */
    int reader_loop();

private:
    // 运行时选项
    const char *_device{"/dev/ttyS1"};
    int _baudrate{115200};
    bool _simulate{false};
    uint32_t _sim_interval_us{20000}; // 默认 50 Hz

    // 串口 fd
    int _fd{-1};

    // uORB 发布器
    uORB::Publication<force_sensor_s> _pub{ORB_ID(force_sensor)};

    // 帧缓冲
    char _frame_buf[32]{}; // 够 17 字节
    size_t _frame_len{0};

    // 实机阻塞读线程
    px4_task_t _reader_task{-1};
    volatile bool _exit_requested{false};
};

// 单例指针用于 trampoline 访问实例
static ForceSensor *g_instance{nullptr};

/* ============================= 实现 ============================= */

ForceSensor::ForceSensor()
    : ModuleParams(nullptr)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) // 仅 SIM 用
{
}

ForceSensor::~ForceSensor()
{
    // 请求退出：SIM 清调度；实机关闭 fd 唤醒阻塞读
    _exit_requested = true;
    ScheduleClear();

    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

int ForceSensor::configure_port(int fd, speed_t baud)
{
    struct termios cfg{};
    if (tcgetattr(fd, &cfg) < 0) {
        PX4_ERR("tcgetattr: %d", errno);
        return -errno;
    }

    cfsetispeed(&cfg, baud);
    cfsetospeed(&cfg, baud);

    // 原始模式 8N1，无软硬流控
    cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    cfg.c_cflag |= (CS8 | CLOCAL | CREAD);
    cfg.c_iflag &= ~(IXON | IXOFF | IXANY);
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cfg.c_oflag &= ~OPOST;

    // 读策略：配合 poll()，让 read() 立即取缓冲区内已有字节，不再额外阻塞
    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &cfg) < 0) {
        PX4_ERR("tcsetattr: %d", errno);
        return -errno;
    }
    return PX4_OK;
}

int ForceSensor::init()
{
    if (_simulate) {
        // 仿真模式：按频率定时喂帧
        ScheduleOnInterval(_sim_interval_us);
        PX4_INFO("force_sensor SIM @ %.1f Hz", 1e6 / double(_sim_interval_us));
        return PX4_OK;
    }

    // 实机：打开串口（阻塞模式；不加 O_NONBLOCK）
    _fd = ::open(_device, O_RDWR | O_NOCTTY);
    if (_fd < 0) {
        PX4_ERR("open %s failed: %d", _device, errno);
        return -errno;
    }

    if (configure_port(_fd, map_baud(_baudrate)) != PX4_OK) {
        return PX4_ERROR;
    }

    // 启动读线程（阻塞读），不要在工作队列里阻塞
    g_instance = this;
    _reader_task = px4_task_spawn_cmd(
        MODULE_NAME,                // 任务名
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,     // 如需更高优先级可调整
        2000,                       // 栈
        (px4_main_t)&ForceSensor::reader_trampoline,
        nullptr);

    if (_reader_task < 0) {
        PX4_ERR("spawn reader failed");
        return PX4_ERROR;
    }

    PX4_INFO("force_sensor (blocking) dev=%s baud=%d", _device, _baudrate);
    return PX4_OK;
}

void ForceSensor::Run()
{
    // 仅仿真模式会调度到这里
    if (!_simulate) {
        return;
    }

    static uint32_t t = 0;
    uint16_t s1 = (1000 + (t % 9000)) % 10000;
    uint16_t s2 = (2000 + ((t * 3) % 9000)) % 10000;
    uint16_t s3 = (3000 + ((t * 7) % 9000)) % 10000;
    uint16_t s4 = (4000 + ((t * 11) % 9000)) % 10000;

    char frame[18] = {0};
    // snprintf 会写入终止符 '\0'，我们只喂 17 个有效字节
    snprintf(frame, sizeof(frame), "R%04u%04u%04u%04u", s1, s2, s3, s4);
    handle_bytes(frame, 17);

    t++;
}

int ForceSensor::reader_trampoline(int, char**)
{
    return g_instance ? g_instance->reader_loop() : -1;
}

int ForceSensor::reader_loop()
{
    struct pollfd pfd{};
    pfd.fd = _fd;
    pfd.events = POLLIN;

    char buf[64];

    while (!_exit_requested && !should_exit()) {
        int pret = ::poll(&pfd, 1, 100 /*ms*/);

        if (pret > 0 && (pfd.revents & POLLIN)) {
            ssize_t n = ::read(_fd, buf, sizeof(buf));
            if (n > 0) {
                handle_bytes(buf, (size_t)n);
            }
        } else if (pret == 0) {
            // 超时：检查退出标志后继续
        } else if (pret < 0) {
            if (errno == EINTR) {
                continue;
            }
            PX4_ERR("poll err: %d", errno);
            px4_usleep(10000);
        }
    }

    return 0;
}

/* ========================== ModuleBase 钩子 ========================== */

int ForceSensor::task_spawn(int argc, char *argv[])
{
    // 默认参数
    const char *device = "/dev/ttyS1";
    int baud = 115200;
    bool sim = false;
    unsigned sim_hz = 50;

    // 解析命令行（位于 "start" 之后）
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) && (i + 1) < argc) {
            device = argv[++i];
        } else if ((strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) && (i + 1) < argc) {
            baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--sim") == 0) {
            sim = true;
            if ((i + 1) < argc) {
                char *endp = nullptr;
                long v = strtol(argv[i + 1], &endp, 10);
                if (endp && *endp == '\0' && v > 0) {
                    sim_hz = (unsigned)v;
                    i++;
                }
            }
        }
    }

    ForceSensor *instance = new ForceSensor();
    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    instance->_device = device;
    instance->_baudrate = baud;
    instance->_simulate = sim;
    instance->_sim_interval_us = (sim_hz > 0) ? (1000000u / sim_hz) : 20000u;

    _object.store(instance);
    _task_id = -1; // 非 pthread，工作队列不用于实机模式

    int ret = instance->init();
    if (ret != PX4_OK) {
        delete instance;
        _object.store(nullptr);
        return ret;
    }

    return PX4_OK;
}

ForceSensor *ForceSensor::instantiate(int, char*[])
{
    return nullptr; // 不使用
}

int ForceSensor::print_usage(const char *reason)
{
    if (reason) {
        PX4_ERR("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESC(
### Description
Reads frames from a serial device ('R' + 16 ASCII digits) and publishes four parsed values on `force_sensor`.

### Protocol
Each frame: `Rdddddddddddddddd`  // 4 x 4-digit decimal numbers

### Modes
- Real device: dedicated thread with blocking poll/read (low CPU)
- Simulation: ScheduledWorkItem at given Hz feeding the same parser

### Usage
force_sensor start [-d <device>] [-b <baud>] [--sim [Hz]]
  -d, --device : serial path (default /dev/ttyS1)
  -b, --baud   : baudrate (default 115200)
  --sim [Hz]   : run in simulation mode at given rate (default 50 Hz)

force_sensor status
force_sensor stop
)DESC");

    PRINT_MODULE_USAGE_NAME("force_sensor", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", nullptr, "Serial device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 2000000, "Baud rate", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('S', "Simulation mode (alias: --sim)", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int ForceSensor::custom_command(int, char*[])
{
    return print_usage("unrecognized command");
}

extern "C" __EXPORT int force_sensor_main(int argc, char *argv[])
{
    return ForceSensor::main(argc, argv);
}

// 可选别名，部分环境会用 *_app_main 入口
extern "C" __EXPORT int force_sensor_app_main(int argc, char *argv[])
{
    return force_sensor_main(argc, argv);
}
