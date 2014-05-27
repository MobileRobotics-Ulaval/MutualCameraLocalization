##MutualCameraLocalization##

3D localization system based on 2 cameras and a minimal set of markers. The code is largely based on [RPG Monocular Pose Estimator](https://github.com/uzh-rpg/rpg_monocular_pose_estimator). The dot finder use multiple algorithm the filter out the very noisy environnment.

##clientCube##
This ROS node convert infrared image from the image server to a ROS *image_raw* and *camera_info message*. The client can send command to the image server and change the camera's parameters. The node need the parameters: 
 * *cubeid* : change the name of the node
 * *address* : Ip address of the server
 * *port* : The port of the TCP socket


##ImgServer##
This is not a ROS node, it's a image server that connect take picture from a camera using the [Flycapture2 SDK](http://ww2.ptgrey.com/sdk/flycap) and broadcast them using TCP/IP socket. The parameters of the camera can be change by the client. Each black and white Infrared image is thresholded, compressed with [lZ4](https://code.google.com/p/lz4/) and send serialize with a protobuffer. Binary for the protobuf librairy on ARM is provided. The Flycapture driver need to be install on the system.

To compile ImgServer:
```bash
cd imgServer/src
make
../FlyCamServer
```
##sick_pose##
Sick_pose analyse the LaserScan from a Sick LMS2xx to find the position and orientation of the cubes. The node return the position of the cube A and cube B with a [PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) message. The algorithm filter out the image to find the two points clusters corresponding to the cubes.


##tcpPoseStream##
This ROS node subcribes to the PoseStamped of the sick_pose and send them throught a socket to the cube controler by a TCP/IP Socket. This node takes the parameters:
 * *topic* : Name of the PoseStamped topic to  listen to
 * *address* : Ip address of the server
 * *port* : The port of the TCP socket
