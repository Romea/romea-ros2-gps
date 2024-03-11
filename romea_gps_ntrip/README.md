This package provides nodes to connect to a GNSS via TCP clients or a serial port to get NMEA
sentences from the GNSS and send RTCM data to the GNSS from a NTRIP client or a TCP client.

## Installation

The required python dependencies can be installed using pip:
```
pip install -r requirements.txt
```

After that, you can use the standard compilation process of ROS2.


## Running

### Connect a GNSS to caster.centipede.fr using TCP clients

If you have a GNSS with the following interfaces:

* the GNSS has the following IP address: 192.168.0.50
* NMEA data are published by a TCP server on port 1001
* RTCM data are listened by a TCP server on port 1002

then, you can connect it to the NTRIP caster.centipede.fr using the following command:

```
ros2 launch romea_gps_ntrip centipede_tcp.launch.py
```

When started, the topic `/gps/nmea_sentence` will contain the NMEA sentences published by the GNSS.
You can specify another IP or TCP ports by passing parameters to the launch (list available using
`--show-args`)


### Manually connect a GNSS caster.centipede.fr using serial port

If you have a GNSS with the following interfaces:

* the GNSS is detected as a serial port named '/dev/ttyUSB0' at 115200 bit/s
* NMEA data are published by the GNSS on the serial port
* RTCM data are listened by the GNSS on the serial port

then, you can connect it to the NTRIP caster.centipede.fr using the following command:

```
ros2 run romea_gps_ntrip serial_nmea_ntrip --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200
```
