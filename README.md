# gps_publisher

To run it.

On your pc:
```
colcon build
ros2 run gps_publisher gps_publisher_node 
```

On Cohda:
```
socat SYSTEM:"gpspipe -r" UDP:172.16.1.1:5000
```
