# force_sensor PX4 module

This example shows how to create a simple PX4 module and uORB message for
reading sensor data from a serial port (TELEM1 on a CUAV V5+ flight
controller) running PX4 v1.15 firmware.  The data coming from the
external device has the form:

```
R0000015000000150
```

The letter `R` is a frame header.  It is followed by four
four‑character ASCII strings representing the decimal readings of four
sensors.  For example, `0150` means the fourth sensor has the value
`150`.

The module does the following:

1. **Opens the TELEM 1 UART** (on CUAV V5+ this is usually `/dev/ttyS1`) at
   the desired baud rate (115200 baud by default).
2. **Parses incoming frames** that start with `'R'` followed by 16 ASCII
   digits.  Each group of four digits is converted into a 16‑bit
   integer (sensor 1 to sensor 4).
3. **Publishes the data** on a custom uORB topic (`force_sensor`)
   containing a timestamp and the four sensor values.  The message
   definition includes the mandatory `timestamp` field; the uORB
   documentation notes that all message definitions must include this
   field so that the logger can record the topic【276250905605543†L1894-L1899】.
4. The PX4 **system logger can log any uORB topic**; specifying the
   topic name (and optional interval) is sufficient【38122509039458†L1822-L1825】.  Once
   compiled into the firmware and added to the list of logged topics
   (`logger add force_sensor 0` or via the `logger_topics.txt` file on
   the SD card), the values will appear in the ULog file.
5. Optionally prints messages to the PX4 shell using `PX4_INFO`.  The
   PX4 logging tutorial notes that warnings and errors are automatically
   added to the ULog【52272034270050†L1935-L1938】.

## Building as an out‑of‑tree module

PX4 supports adding external modules and messages out of tree.  The
`force_sensor_module` directory is structured accordingly:

```
force_sensor_module/
├── CMakeLists.txt
├── msg/
│   ├── CMakeLists.txt
│   └── force_sensor.msg
└── src/
    └── modules/
        └── force_sensor/
            ├── CMakeLists.txt
            └── force_sensor.cpp
```

Follow these steps to build the module with PX4 v1.15:

1. Clone the PX4‑Autopilot repository and check out the `v1.15.0` tag
   or the stable branch.
2. Copy the `force_sensor_module` directory somewhere outside the PX4
   tree, for example to `~/px4_external`.
3. Build using:

   ```sh
   cd PX4-Autopilot
   make px4_fmu-v6x_default EXTERNAL_MODULES_LOCATION=~/px4_external/force_sensor_module
   ```

   Replace `px4_fmu-v6x_default` with the target for your board (e.g.
   `px4_fmu-v5_default` for CUAV V5+).  The `EXTERNAL_MODULES_LOCATION`
   variable tells CMake where to find external modules and messages.
4. Flash the firmware using the usual `make <board>_default upload`
   command.
5. On the PX4 shell, start the module with:

   ```
   force_sensor start
   ```

   Use `force_sensor status` to confirm it is running and
   `force_sensor stop` to terminate.

6. To log the custom topic, either run `logger add force_sensor 0` on
   the shell before arming, or create a `etc/logging/logger_topics.txt`
   file on the SD card containing:

   ```
   force_sensor 0 0
   ```

   The first `0` tells the logger to log every message (no interval
   limit) and the second `0` logs all instances of the topic.

After a flight, download the ULog file and inspect it using Flight
Review or `pyulog`.  The custom `force_sensor` topic will contain the
timestamp and four sensor values.

## build command: 
HEADLESS=1 make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION=/home/sia/px4_custom_module
make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=/home/sia/px4_custom_module

## 测试（SITL）:
### 进入 PX4 控制台后，启动仿真数据源：
force_sensor start --sim 100   # 100 Hz 例子
listener force_sensor          # 看看是否在发布
### 伪串口自测:
1. sudo apt-get install socat
2. socat -d -d pty,raw,echo=0 pty,raw,echo=0
记下两端的路径，比如 /dev/pts/5 和 /dev/pts/7
3. 模块指向一端：
force_sensor start -d /dev/pts/5 -b 115200
4. 另一端写入帧进行测试：
send once:
echo '01030800110022003301621CA7' | xxd -r -p > /dev/pts/8
send circle:
while true; do echo '01030800110022003301621CA7' | xxd -r -p > /dev/pts/7; sleep 0.05; done 

echo -n "R0001000200030004" > /dev/pts/8