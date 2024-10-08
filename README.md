# gps_publisher

ROS2 package that grabs NMEA sentences from a GPS accessible through UDP (port 5000 by default) and publishes in a ROS2 Node.

## To add to a ROS2 workspace
Preferably clone it into your workspace's src folder

## To run it

**On the root folder of the ROS2 workspace on your pc:**
```
colcon build --packages-select gps_publisher
source install/setup.bash
ros2 run gps_publisher gps_publisher_node 
```

This will start the **gps\_publisher** node and create a listener on the UDP port 5000

**On Cohda:**

> Change the IP address according to the device that should listen to the UDP port 5000

```
socat SYSTEM:"gpspipe -r" UDP:172.16.1.1:5000
```

This will forward the output of "gpspipe -r", which are the NMEA sentences, to the UDP port 5000 of the device with IP 172.16.1.1

## Topic publishing

The **gps\_publisher** node publishes the converted latitude and longitude (in degrees) into the **gps\_coordinates** topic 

Thus, you can monitor it through:

```
ros2 topic echo \gps_coordinates
```
