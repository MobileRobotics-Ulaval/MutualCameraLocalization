#/bin/bash

gnome-terminal -x bash -c "rosrun sicktoolbox_wrapper sicklms _port:=\"/dev/ttyUSB0\"" &
gnome-terminal -x bash -c "rosrun sick_pose sick_pose _path:=\"\"" &
gnome-terminal -x bash -c "rosrun rviz rviz" &
gnome-terminal -x bash -c "rqt" &


gnome-terminal -x bash -c "rosrun tcp_pose_stream tcp_pose_stream _address:=\"192.168.10.244\" _port:=50000 _topic:=\"/cubeA_pose\"" &
gnome-terminal -x bash -c "rosrun tcp_pose_stream tcp_pose_stream _address:=\"192.168.10.243\" _port:=50000 _topic:=\"/cubeB_pose\"" &


