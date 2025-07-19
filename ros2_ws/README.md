# EMG Sensor ROS 2 Bridge

``ble_bridge.py`` connects to the nRF52840 DK running `peripheral_uart`
and republishes the byte stream on **/emg_raw** as `std_msgs/UInt8MultiArray`.

```bash
# Jetson
source /opt/ros/humble/setup.bash
cd ~/Github/EMG-Project/ros2_ws
colcon build
source install/setup.bash
ros2 run emg_sensor ble_bridge
