sudo chmod 666 /dev/ttyUSB0

colcon build
source install/setup.bash

gnome-terminal -- ros2 run uwb_aoa_pkg libAoa_robot_example "/dev/ttyUSB0"

sleep 2

NODE_NAME="/libAoa_robot_publisher"

if ros2 node list | grep -q "$NODE_NAME"; then
  ros2 run plotjuggler plotjuggler  --layout ./libaoa_example_layout.xml
else
  echo "Error! Check uart connection/permission!"
fi

