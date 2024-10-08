# gps_publisher

ROS2 package that grabs NMEA sentences from a GPS accessible through UDP (port 5000 by default) and publishes in a ROS2 Node.

## To run it

**On your pc:**
```
colcon build
ros2 run gps_publisher gps_publisher_node 
```

**On Cohda:**

> Change the ip according to the device that should listen to the UDP port 5000

```
socat SYSTEM:"gpspipe -r" UDP:172.16.1.1:5000
```
