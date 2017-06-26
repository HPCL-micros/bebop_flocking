### Configure ROS remote launch
Take uav0 with ip address 192.168.21.100 as an example.
Roscore is running on the workstation with ip address 192.168.21.200.

#### Edit .bashrc
```
export ROS_MASTER_URI=http://192.168.21.200:11311 
export ROS_HOSTNAME=192.168.21.100
export ROSLAUNCH_SSH_UNKNOWN=1
```

#### Create env.sh in /home/pi/catkin_ws/
```
# !/bin/bash
. /home/pi/catkin_ws/devel/setup.sh
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME="192.168.21.100"
exec "$@"
```
#### Mark env.sh with executable permission.
```
chmod +x /home/pi/catkin_ws/env.sh
```
### SD card image
Dowload from http://pan.baidu.com/s/1i4PlIT3 .
It is a good choice to write the SD card image to your SD card with Win32DiskImager. Then just turn on the Raspberry PI with the SD card and everthing is set. \
This image contains Ubuntu Mate with ROS and our code. The image is generated from our uav0. For other uavs, just change the ip address in file ~/.bashrc and ~/catkin_ws/env.sh. The user name and password are both `pi`. \
Note that the system is configured to connect our wifi automatically after booting, so you can not select a wifi network. You may wish to edit /etc/network/interfaces to change this setting.
```
sudo vim /etc/network/interfaces
```
