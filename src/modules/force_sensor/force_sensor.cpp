/*
 * force_sensor.cpp
 * 1. force_sensor_main 启动并调用 task_spawn。
 * 2. task_spawn 创建并初始化 ForceSensor 实例，调用 init 来启动串口或仿真。
 * 3. init 配置串口或定时任务，并开始定时调用 Run。
 * 4. Run 通过读取串口或生成仿真数据来调用 handle_bytes 解析数据。
 * 5. 数据通过 uORB 发布
 *
 * A PX4 module that reads a simple frame-based protocol from a serial
 * port and publishes the values on a custom uORB topic. Now supports:
 *  - configurable device and baudrate via CLI (not just TELEM1)
 *  - simulation mode that generates real-like frames and feeds
 *    the same parser for SITL/self-test.
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/force_sensor.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

using namespace time_literals;

class ForceSensor : public ModuleBase<ForceSensor>,
                    public ModuleParams,
                    public px4::ScheduledWorkItem
{
public:
    ForceSensor();
    virtual ~ForceSensor() override;

    int init();

    static int task_spawn(int argc, char *argv[]);
    static ForceSensor *instantiate(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);

private:
    void Run() override;
    int configure_port(int fd, speed_t baud);

    // map integer baud to termios speed_t
    static speed_t map_baud(int baudrate) {
        switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
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
        default: return B115200; // fallback
        }
    }

    // common byte handler so real+sim 复用同一套解析与发布逻辑
    void handle_bytes(const char *data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            char c = data[i];
            if (_frame_len == 0) {
                if (c == 'R') { _frame_buf[_frame_len++] = c; }
            } else {
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
                        case 0: msg.sensor1 = (uint16_t)val; break;
                        case 1: msg.sensor2 = (uint16_t)val; break;
                        case 2: msg.sensor3 = (uint16_t)val; break;
                        case 3: msg.sensor4 = (uint16_t)val; break;
                        }
                    }
                    _pub.publish(msg);
                    _frame_len = 0; // next frame
                }
            }
        }
    }

private:
    // runtime options
    const char *_device{"/dev/ttyS1"};
    int _baudrate{115200};
    bool _simulate{false};
    uint32_t _sim_interval_us{20000}; // default 50 Hz
    // serial fd
    int _fd{-1};
    // uORB pub
    uORB::Publication<force_sensor_s> _pub{ORB_ID(force_sensor)};
    // frame buffer
    char _frame_buf[32]{};
    size_t _frame_len{0};
};

ForceSensor::ForceSensor()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ForceSensor::~ForceSensor()
{
    ScheduleClear();
    if (_fd >= 0) { ::close(_fd); _fd = -1; }
}

int ForceSensor::configure_port(int fd, speed_t baud)
{
    struct termios config{};
    if (tcgetattr(fd, &config) < 0) {
        PX4_ERR("tcgetattr failed: %d", errno);
        return -errno;
    }

    cfsetispeed(&config, baud);
    cfsetospeed(&config, baud);

    // 8N1
    config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    config.c_cflag |= (CS8 | CLOCAL | CREAD);

    // no flow control
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    // raw
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &config) < 0) {
        PX4_ERR("tcsetattr failed: %d", errno);
        return -errno;
    }
    return 0;
}

int ForceSensor::init()
{
    if (_simulate) {
        // 仿真：不打开串口，定时生成“R+16位数字”喂给解析器
        ScheduleOnInterval(_sim_interval_us);
        PX4_INFO("force_sensor SIM mode @ %.1f Hz", 1e6 / static_cast<double>(_sim_interval_us));
        return PX4_OK;
    }

    // 打开指定设备
    _fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        PX4_ERR("open %s failed: %d", _device, errno);
        return -errno;
    }

    if (configure_port(_fd, map_baud(_baudrate)) != 0) {
        return PX4_ERROR;
    }

    // 5ms 轮询串口
    ScheduleOnInterval(5_ms);
    PX4_INFO("force_sensor started, dev=%s baud=%d", _device, _baudrate);
    return PX4_OK;
}

void ForceSensor::Run()
{
    if (_simulate) {
        // 生成一个平滑变化的4通道数据（0..9999 循环），并以真实帧格式喂给解析器
        static uint32_t t = 0;
        uint16_t s1 = (1000 + (t % 9000)) % 10000;
        uint16_t s2 = (2000 + ((t * 3) % 9000)) % 10000;
        uint16_t s3 = (3000 + ((t * 7) % 9000)) % 10000;
        uint16_t s4 = (4000 + ((t * 11) % 9000)) % 10000;
        char frame[18] = {0}; // 'R' + 16 digits + '\0'
        // 保证每个值4位，前导0补齐
        // 注意：snprintf 末尾会写 '\0'，但我们只喂 17 个字节给解析器
        snprintf(frame, sizeof(frame), "R%04u%04u%04u%04u", s1, s2, s3, s4);
        handle_bytes(frame, 17);
        t++;
        return;
    }

    if (_fd < 0) { return; }

    char buf[32];
    ssize_t nread = ::read(_fd, buf, sizeof(buf));
    if (nread > 0) {
        handle_bytes(buf, (size_t)nread);
    }
}

int ForceSensor::task_spawn(int argc, char *argv[])
{
    // 缺省值
    const char *device = "/dev/ttyS1";
    int baud = 115200;
    bool sim = false;
    unsigned sim_hz = 50;

    // 解析命令行参数（在 'start' 之后）
    // 支持：
    //   -d|--device <path>
    //   -b|--baud <int>
    //   --sim [hz]
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

    // 将解析到的配置写入实例
    instance->_device = device;
    instance->_baudrate = baud;
    instance->_simulate = sim;
    instance->_sim_interval_us = (sim_hz > 0) ? (1000000u / sim_hz) : 20000u;

    _object.store(instance);
    _task_id = -1; // work-queue task (no pthread)

    int ret = instance->init();
    if (ret != PX4_OK) {
        delete instance;
        _object.store(nullptr);
        return ret;
    }
    return PX4_OK;
}

ForceSensor *ForceSensor::instantiate(int, char *[])
{
    return nullptr; // not used
}

int ForceSensor::print_usage(const char *reason)
{
    if (reason) {
        PX4_ERR("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"desc(
### Description

Reads frames from a serial device and publishes four parsed sensor values
on the `force_sensor` uORB topic.

### Protocol

Each frame: `Rdddddddddddddddd` ('R' + 16 ASCII digits = 4 x 4-digit values).

### Features

- Configurable device/baud via CLI
- Simulation mode (no serial) that generates real-like frames and feeds the same parser

### Usage

force_sensor start [-d <device>] [-b <baud>] [--sim [Hz]]
  -d, --device : serial device path (default: /dev/ttyS1)
  -b, --baud   : baudrate (default: 115200)
  --sim [Hz]   : run in simulation mode at given rate (default 50 Hz)

force_sensor status
force_sensor stop

Example:
  force_sensor start                           # TELEM1 @115200
  force_sensor start -d /dev/ttyS2 -b 921600   # use TELEM2 @921600
  force_sensor start --sim                     # sim at 50 Hz
  force_sensor start --sim 200                 # sim at 200 Hz
)desc"
    );

    PRINT_MODULE_USAGE_NAME("force_sensor", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", nullptr, "Serial device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 10000000, "Baud rate", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('S', "Simulation mode (alias: --sim)", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int ForceSensor::custom_command(int, char *[])
{
    return print_usage("unrecognized command");
}

extern "C" __EXPORT int force_sensor_main(int argc, char *argv[])
{
    return ForceSensor::main(argc, argv);
}

extern "C" __EXPORT int force_sensor_app_main(int argc, char *argv[])
{
    return force_sensor_main(argc, argv);
}
