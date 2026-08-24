把uwb文件夹放在workspace/src/中
定位结果画图使用plotjuggler,需要提前安装，安装方式 sudo apt install ros-humble-plotjuggler-ros

测试用例使用./start_libAoa_robot.sh，它进行编译，run publisher节点和画图三个操作。

	编译默认使用colcon，若并未安装，安装方式 sudo apt install python3-colcon-common-extensions
	运行节点时默认新打开一个终端观察定位数据，使用gnome-terminal，若并未安装，安装方式sudo apt-get install gnome-terminal

串口名称默认 "/dev/ttyUSB0"，可在start_libAoa_robot.sh内改变

链接串口，启动对手件测距，运行 ./start_libAoa_robot.sh 即可启动测试（为了画图需在弹出窗口点击两次确认，一次确认使用topic subscriber，一次选择libaoa_robot_publisher节点画图）

start_libAoa_robot.sh内：
  ros2 run uwb_aoa_pkg libAoa_robot_example "/dev/ttyUSB0"开启名为"/libAoa_robot_publisher"的节点，发布名为"/libAoa_robot_publisher"的topic,包含./msg文件夹内定义的msg信息，包含：
  -定位极坐标下的r,a，
  -xy坐标下的x,y，
  -算法状态state,
  -信号强度rssi，
  -置信度pos_confidence

  plot_juggler 订阅"/libAoa_robot_publisher" topic 并画图
