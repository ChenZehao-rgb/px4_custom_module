# Force sensor readings
#
# This custom uORB message is used by the force_sensor module to publish
# sensor values read from an external device connected via the TELEM1
# port. A timestamp is required so that the PX4 logger can record the topic.

uint64 timestamp               # [us] Time since system boot.
uint16 sensor1                 # Sensor 1 reading (0-9999)
uint16 sensor2                 # Sensor 2 reading (0-9999)
uint16 sensor3                 # Sensor 3 reading (0-9999)
uint16 sensor4                 # Sensor 4 reading (0-9999)

uint8 ORB_QUEUE_LENGTH = 8     # allow a small queue in case of delays
