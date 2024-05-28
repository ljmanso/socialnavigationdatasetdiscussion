source /opt/ros/humble/setup.bash

mkdir -p rclpy_ws/src
cd rclpy_ws/src
git clone https://github.com/ros2/rclpy.git
cd ..
colcon build --symlink-install

source install/setup.bash
cd src/rclpy/rclpy/docs
make html


