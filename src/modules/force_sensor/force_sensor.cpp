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

#include <uORB/topics/debug_array.h> // For debug

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
    // void handle_bytes(const char *data, size_t len) {
    //     for (size_t i = 0; i < len; i++) {
    //         char c = data[i];

    //         if (_frame_len == 0) {
    //             if (c == 'R') {
    //                 _frame_buf[_frame_len++] = c;
    //             }
    //             continue;
    //         }

    //         if (_frame_len < sizeof(_frame_buf)) {
    //             _frame_buf[_frame_len++] = c;
    //         }

    //         if (_frame_len == 17) {
    //             msg.timestamp = hrt_absolute_time();

    //             for (int j = 0; j < 4; j++) {
    //                 char digits[5] = {0};
    //                 memcpy(digits, &_frame_buf[1 + j * 4], 4);
    //                 unsigned long val = strtoul(digits, nullptr, 10);
    //                 switch (j) {
    //                     case 0: msg.sensor1 = static_cast<uint16_t>(val); break;
    //                     case 1: msg.sensor2 = static_cast<uint16_t>(val); break;
    //                     case 2: msg.sensor3 = static_cast<uint16_t>(val); break;
    //                     case 3: msg.sensor4 = static_cast<uint16_t>(val); break;
    //                 }
    //             }

    //             _pub.publish(msg);
    //             if (_use_debug)
    //             {
    //                 _debug_msg.timestamp = msg.timestamp;
    //                 _debug_msg.data[0] = msg.sensor1;
    //                 _debug_msg.data[1] = msg.sensor2;
    //                 _debug_msg.data[2] = msg.sensor3;
    //                 _debug_msg.data[3] = msg.sensor4;
    //                 _debug_pub.publish(_debug_msg);
    //             }
    //             _frame_len = 0; // 下一帧
    //         }
    //     }
    // }

    // 取 BE/LE 32 位有符号
    static inline int32_t be_i32(const uint8_t *p) {
        return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                        ((uint32_t)p[2] << 8)  |  (uint32_t)p[3]);
    }
    static inline int32_t le_i32(const uint8_t *p) {
        return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
    }

    static inline void shift_buffer(uint8_t *buf, size_t &len, size_t n) {
        if (n >= len) { len = 0; return; }
        memmove(buf, buf + n, len - n);
        len -= n;
    }

    void handle_bytes(const char *data, size_t len)
    {
        const uint8_t *in = reinterpret_cast<const uint8_t*>(data);

        for (size_t i = 0; i < len; i++) {
            if (_frame_len < sizeof(_frame_buf)) _frame_buf[_frame_len++] = in[i];
            else { shift_buffer(_frame_buf, _frame_len, 1); _frame_buf[_frame_len++] = in[i]; }

            while (_frame_len >= 24) {
                // 同步帧头 0x01 0x50
                size_t start = 0;
                while (start + 1 < _frame_len) {
                    if (_frame_buf[start] == 0x01 && _frame_buf[start + 1] == 0x50) break;
                    start++;
                }
                if (start) shift_buffer(_frame_buf, _frame_len, start);
                if (_frame_len < 24) break;

                // 检查帧尾 0xFF 0xFE
                if (!(_frame_buf[22] == 0xFF && _frame_buf[23] == 0xFE)) {
                    shift_buffer(_frame_buf, _frame_len, 1);
                    continue;
                }

                // 数据区（头后到尾前）：第 2..21 字节（共 20B）
                const uint8_t *payload = &_frame_buf[2]; // 长度 20B：可能含对齐字节 + 16B通道 + 4B未知
                int32_t v[4] = {0};
                bool parsed = false;

                // 1) 自适应：尝试 0..3 的起始偏移，按 BE32
                for (int off = 0; off <= 3 && !parsed; off++) {
                    // 要保证 off+16 <= 20（四个通道 16B 都在 payload 内）
                    if (off + 16 > 20) break;

                    // 判定“最高字节是否像符号扩展”（全 0 或全 FF）
                    bool ok = true;
                    for (int k = 0; k < 4; k++) {
                        uint8_t msb = payload[off + k*4 + 0]; // BE32 的最高字节
                        if (!(msb == 0x00 || msb == 0xFF)) { ok = false; break; }
                    }
                    if (!ok) continue;

                    for (int k = 0; k < 4; k++) v[k] = be_i32(payload + off + k*4);
                    parsed = true;
                }

                // 2) 兜底：按原先 LE32@偏移0
                if (!parsed) {
                    v[0] = le_i32(payload + 0);
                    v[1] = le_i32(payload + 4);
                    v[2] = le_i32(payload + 8);
                    v[3] = le_i32(payload + 12);
                }

                // 发布
                msg.timestamp = hrt_absolute_time();
                msg.sensor1 = v[0];
                msg.sensor2 = v[1];
                msg.sensor3 = v[2];
                msg.sensor4 = v[3];
                _pub.publish(msg);

                if (_use_debug) {
                    _debug_msg.timestamp = msg.timestamp;
                    _debug_msg.data[0] = msg.sensor1;
                    _debug_msg.data[1] = msg.sensor2;
                    _debug_msg.data[2] = msg.sensor3;
                    _debug_msg.data[3] = msg.sensor4;
                    _debug_pub.publish(_debug_msg);
                }

                // 消费整帧
                shift_buffer(_frame_buf, _frame_len, 24);
            }
        }
    }


    /** 独立线程入口（trampoline） */
    static int reader_trampoline(int argc, char *argv[]);
    /** 阻塞读主循环（poll + read） */
    int reader_loop();

private:
    // 运行时选项
    const char *_device{"/dev/ttyS2"};
    int _baudrate{115200};
    bool _use_debug{false};

    // 串口 fd
    int _fd{-1};

    // uORB 发布器
    uORB::Publication<force_sensor_s> _pub{ORB_ID(force_sensor)};
    uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};

    force_sensor_s msg{};
    debug_array_s _debug_msg{};
    // 帧缓冲
    // char _frame_buf[32]{}; // 够 17 字节
    uint8_t _frame_buf[256]{};
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
}

void ForceSensor::Run()
{
    // 当前模块仅在实机模式下使用阻塞读线程；留出 SIM 扩展点
    if (_exit_requested || should_exit()) {
        ScheduleClear();
        return;
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
    cfg.c_cflag &= ~CRTSCTS;

    cfg.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | BRKINT | PARMRK | INPCK | ISTRIP);
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cfg.c_oflag &= ~(OPOST | ONLCR | OCRNL);

    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &cfg) < 0) {
        PX4_ERR("tcsetattr: %d", errno);
        return -errno;
    }
    tcflush(fd, TCIOFLUSH); // 清空历史残留
    return PX4_OK;
}

int ForceSensor::init()
{
    _debug_msg.id = 1;
    strncpy(_debug_msg.name, "force_sensor", sizeof(_debug_msg.name));

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

    _task_id = _reader_task; // 标记为已启动
    PX4_INFO("force_sensor (blocking) dev=%s baud=%d", _device, _baudrate);
    return PX4_OK;
}

int ForceSensor::reader_trampoline(int, char**)
{
    return g_instance ? g_instance->reader_loop() : -1;
}

int ForceSensor::reader_loop()
{
    // 在子任务里打开串口（很关键）
    int fd = ::open(_device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        PX4_ERR("open %s failed: %d", _device, errno);
        return -errno;
            }
    if (configure_port(fd, map_baud(_baudrate)) != PX4_OK) {
        ::close(fd);
        return PX4_ERROR;
    }
    // 记录给调试/状态用；注意：不同 task 的 fd 表不共享，父任务关不掉这个 fd
    _fd = fd;

    char buf[64];

    while (!_exit_requested && !should_exit()) {
        ssize_t n = ::read(fd, buf, sizeof(buf)); // 最多阻塞 0.5s（由 VTIME 决定）
        if (n > 0) {
            handle_bytes(buf, n);
        } else if (n == 0) {
            // 超时，检查退出标志后继续
        } else if (errno != EINTR) {
            PX4_ERR("read err: %d", errno);
            px4_usleep(10000);
        }
    }
    ::close(fd);
    _fd = -1;
    return 0;
}

/* ========================== ModuleBase 钩子 ========================== */

int ForceSensor::task_spawn(int argc, char *argv[])
{
    // 默认参数
    const char *device = "/dev/ttyS2";
    int baud = 115200;
    bool use_debug = false;

    // 解析命令行（位于 "start" 之后）
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) && (i + 1) < argc) {
            device = argv[++i];
        } else if ((strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) && (i + 1) < argc) {
            baud = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--use_debug") == 0) && (i + 1) < argc) {
            use_debug = (strcmp(argv[++i], "true") == 0);
        }
    }

    ForceSensor *instance = new ForceSensor();
    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    instance->_device = device;
    instance->_baudrate = baud;
    instance->_use_debug = use_debug;

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
- Real device: dedicated thread with blocking read (low CPU)

### Usage
force_sensor start [-d <device>] [-b <baud>] [--sim [Hz]]
  -d, --device : serial path (default /dev/ttyS1)
  -b, --baud   : baudrate (default 115200)
  -u, --use_debug : publish debug_array topic (default false)

force_sensor status
force_sensor stop
)DESC");

    // use example to print parameter info
    PRINT_MODULE_USAGE_NAME("force_sensor", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", nullptr, "Serial device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 2000000, "Baud rate", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('u', "Publish debug_array topic (alias: --use_debug)", true);
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
