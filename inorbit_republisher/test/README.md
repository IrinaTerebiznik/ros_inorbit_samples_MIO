# Sample data for testing

A very simple ``rosbag`` with hardcoded data for smoke testing the transformer node.

```bash
$root@7c94d2cf9659:~# ros2 bag info ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3

closing.
[INFO] [1734553121.938063653] [rosbag2_storage]: Opened database 'ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3' for READ_ONLY.

Files:             ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3
Bag size:          24.0 KiB
Storage id:        sqlite3
Duration:          16.005861016s
Start:             Dec 13 2024 19:45:00.085674077 (1734119100.085674077)
End:               Dec 13 2024 19:45:16.091535093 (1734119116.091535093)
Messages:          34
Topic information: Topic: /my_magnetic_field | Type: sensor_msgs/msg/MagneticField | Count: 17 | Serialization Format: cdr
                   Topic: /my_temperature | Type: sensor_msgs/msg/Temperature | Count: 17 | Serialization Format: cdr

```

To validate the node works launch the sample by using the ``sample_data.launch.xml`` launch file and look at ``out`` topic: ``ros2 topic echo my_temperature``.

```bash
cd ros2_ws/src
colcon build --packages-select inorbit_republisher --symlink-install
source install/setup.bash
cd
ros2 launch inorbit_republisher sample_data.launch.xml
# On a different terminal windows
source /opt/ros/jazzy/setup.bash
$ ros2 topic echo my_temperature
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
temperature: 29.068787468933582
variance: 0.0
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
temperature: 22.9644622619558
variance: 0.0
---
```
