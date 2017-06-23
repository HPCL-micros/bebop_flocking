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
