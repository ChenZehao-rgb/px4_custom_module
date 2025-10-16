/*
 * tele1_sensor.cpp
 *
 * A PX4 module that reads a simple frame‑based protocol from a serial
 * port (TELEM1) and publishes the values on a custom uORB topic.  The
 * incoming data frames start with 'R' followed by 16 ASCII digits
 * representing four sensor readings.  The module runs on a scheduled
 * work queue and publishes data periodically.
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/tele1_sensor.h>   // 生成的自定义话题头（tele1_sensor_s）

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <px4_platform_common/time.h>


using namespace time_literals;

/**
 * Tele1Sensor
 *
 * Derives from ModuleBase, ModuleParams and ScheduledWorkItem.  It
 * opens a serial device and reads frames of the form
 *   Rdddddddddddddddd
 * where the 16 digits are split into four sensor values.  Each
 * successfully parsed frame is published on the tele1_sensor uORB topic.
 */
class Tele1Sensor : public ModuleBase<Tele1Sensor>,
                    public ModuleParams,
                    public px4::ScheduledWorkItem
{
public:
    Tele1Sensor();
    virtual ~Tele1Sensor() override;

    /**
     * Initialise the module.  Opens the serial port and schedules
     * periodic execution.
     */
    int init();

    /**
     * Implementation of ModuleBase: spawn the work‑queue task.
     */
    static int task_spawn(int argc, char *argv[]);

    /**
     * Instantiate the module object.  Called from task_spawn().
     */
    static Tele1Sensor *instantiate(int argc, char *argv[]);

    /**
     * Print usage information.
     */
    static int print_usage(const char *reason = nullptr);
    
    static int custom_command(int argc, char *argv[]);


private:
    /**
     * Scheduled work.  Reads from the serial port, parses frames and
     * publishes the data.
     */
    void Run() override;

    /**
     * Configure the serial port (baud rate, format).  Returns 0 on
     * success or a negative error code.
     */
    int configure_port(int fd, speed_t baud);

    // file descriptor for serial port
    int _fd{-1};

    // uORB publication handle
    uORB::Publication<tele1_sensor_s> _pub{ORB_ID(tele1_sensor)};

    // Buffer to accumulate partial frames
    char _frame_buf[32]{};
    size_t _frame_len{0};
};

Tele1Sensor::Tele1Sensor()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

Tele1Sensor::~Tele1Sensor()
{
    // Stop work queue scheduling
    ScheduleClear();

    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

int Tele1Sensor::configure_port(int fd, speed_t baud)
{
    struct termios config{};
    if (tcgetattr(fd, &config) < 0) {
        PX4_ERR("tcgetattr failed: %d", errno);
        return -errno;
    }

    // Input and output baud rate
    cfsetispeed(&config, baud);
    cfsetospeed(&config, baud);

    // 8N1: 8 data bits, no parity, 1 stop bit
    config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    config.c_cflag |= (CS8 | CLOCAL | CREAD);

    // disable flow control
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    // raw input
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &config) < 0) {
        PX4_ERR("tcsetattr failed: %d", errno);
        return -errno;
    }

    return 0;
}

int Tele1Sensor::init()
{
    // open TELEM1 port (CUAV V5+ maps TELEM1 to /dev/ttyS1 on PX4)
    const char *device = "/dev/ttyS1";
    _fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        PX4_ERR("Failed to open %s: %d", device, errno);
        return -errno;
    }

    // configure for 115200 baud, 8N1
    if (configure_port(_fd, B115200) != 0) {
        return PX4_ERROR;
    }

    // schedule periodic execution every 100 ms
    ScheduleOnInterval(100_ms);

    PX4_INFO("tele1_sensor started, reading from %s", device);

    return PX4_OK;
}

void Tele1Sensor::Run()
{
    // if serial port invalid, nothing to do
    if (_fd < 0) {
        return;
    }

    // Read available bytes.  We use a small buffer and non‑blocking
    char buf[32];
    ssize_t nread = ::read(_fd, buf, sizeof(buf));
    if (nread > 0) {
        // iterate through received bytes and assemble frames
        for (ssize_t i = 0; i < nread; i++) {
            char c = buf[i];
            if (_frame_len == 0) {
                // waiting for frame start
                if (c == 'R') {
                    _frame_buf[_frame_len++] = c;
                }
            } else {
                // already have 'R', collect next bytes
                if (_frame_len < sizeof(_frame_buf)) {
                    _frame_buf[_frame_len++] = c;
                }

                // complete frame when we have 'R' + 16 digits
                if (_frame_len == 17) {
                    // parse digits
                    tele1_sensor_s msg{};
                    msg.timestamp = hrt_absolute_time();
                    for (int j = 0; j < 4; j++) {
                        char digits[5] = {0};
                        memcpy(digits, &_frame_buf[1 + j * 4], 4);
                        unsigned long val = strtoul(digits, nullptr, 10);
                        if (j == 0) msg.sensor1 = (uint16_t)val;
                        else if (j == 1) msg.sensor2 = (uint16_t)val;
                        else if (j == 2) msg.sensor3 = (uint16_t)val;
                        else if (j == 3) msg.sensor4 = (uint16_t)val;
                    }

                    // publish message
                    _pub.publish(msg);

                    // reset frame buffer for next frame
                    _frame_len = 0;
                }
            }
        }
    }

    // Reschedule the next run; this is automatically handled by ScheduledWorkItem
    // because ScheduleOnInterval() sets up repeating intervals.
}

int Tele1Sensor::task_spawn(int argc, char *argv[])
{
    Tele1Sensor *instance = new Tele1Sensor();
    if (!instance) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = -1; // work‑queue tasks do not spawn threads

    int ret = instance->init();
    if (ret != PX4_OK) {
        delete instance;
        _object.store(nullptr);
        return ret;
    }
    return PX4_OK;
}

Tele1Sensor *Tele1Sensor::instantiate(int argc, char *argv[])
{
    return nullptr; // not used with ScheduledWorkItem pattern
}

int Tele1Sensor::print_usage(const char *reason)
{
    if (reason) {
        PX4_ERR("%s", reason);
    }
    PRINT_MODULE_DESCRIPTION(
        R"desc(
### Description

Reads frames from a serial device (TELEM1) and publishes the
four parsed sensor values on the `tele1_sensor` uORB topic.

### Implementation

Runs on the high‑priority work queue (hp_default), reading
non‑blocking from `/dev/ttyS1`.  Each frame has the format
`Rdddddddddddddddd` where `d` are digits.  After parsing
the four 4‑digit values, the module publishes them with a
timestamp.  The topic can be logged via the PX4 system logger.

### Usage

```sh
tele1_sensor start    # start the module
tele1_sensor status   # check if running
tele1_sensor stop     # stop the module
```

Add the topic to the logger (for example using `logger add tele1_sensor 0`) to
record the values in the ULog file.

)desc"
    );

    PRINT_MODULE_USAGE_NAME("tele1_sensor", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int Tele1Sensor::custom_command(int argc, char *argv[])
{
    return print_usage("unrecognized command");
}
/**
 * PX4 shell entry point
 */
extern "C" __EXPORT int tele1_sensor_main(int argc, char *argv[])
{
    return Tele1Sensor::main(argc, argv);
}

extern "C" __EXPORT int tele1_sensor_app_main(int argc, char *argv[])
{
    // 一些板卡/配置会寻找 *_app_main，这里转发到 *_main 即可
    return tele1_sensor_main(argc, argv);
}
