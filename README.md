# bebop_flocking
Flocking with bebop 2 and Raspberry Pi 3b. This repository includes both workstation code and drone code (raspberry pi).

## Safety
Conducting experiments with drones may have many risks, including damage to the drones or to nearby people. To Use this code, you should be extremely careful and take your own risk. \
\
Note that this code is based on distributed control, if you lost your connection to the driver on Raspberry PI, in the worst case you may not be able to stop a flying drone.

## Introduction
On the top of each bebop 2, there are a Raspberry Pi and its battery sticking on it, running ROS with Ubuntu Mate. \
The bebop2 ROS driver runs on the Raspberry Pi and connects to the drone with a USB cable. Besides, the flocking algorithm is also running on the Raspberry Pi. \
The workstation (PC) utilize Rviz for visualization and a program to plan path for the drone swarm. \
\
All Raspberry PIs and the workstation are connected via a router(WIFI). Set static IP for these devices. In our case, the workstation's ip address is `192.168.21.200` and the drone's ip address are `192.168.21.100` for uav0, `192.168.21.101` for uav1 etc.

## Localization
In our case, the desired distance between drones is 5m. GPS is not accurate enough. So we utilize the odom data from the drone's ROS driver, which is calculated based on its visual-inertial velocity. \
Since each drone utilize its own odom frame of which the origin is its initial position, we have to mannually relate thier odom frames, namely, `/uav0/odom`，`/uav1/odom` etc., to the world frame`/map`. These tf messages are static and published by `bebop_manger` on the workstation. The initial position of each drone should be set in the roslaunch file `bebop_manager.launch`

## 1. Before taking off
* Turn on all drones and Raspberry PIs.
* Click the botton 4 times for each bebop 2 to enable USB networking.
* Use the workstation to ping each Raspberry PI to check if they are connected. It is also recomanded to ssh to each Raspberry PI and ping `192.168.43.1`, to check the USB connection between the board and the drone.
* Synchronize time for each Raspberry PI. In our case, we run `chrony` on our workstaion and `ntpdate 192.168.21.200` on each Raspberry PI.

## 2. Start roscore on the workstation
```
roscore
```
Important: Never shutdown roscore if there are nodes running on Raspberry PIs, e.g. the drivers. You will not be able to contact the drivers thus losting your control to the drones, which is very dangerous. 

## 3. Launch drivers (six drones for example)
Run the remote roslaunch file from the workstation.
```
roslaunch quadrotor_code driver_six.launch
```
Note that the remote roslaunch enviroment should be set properly.

## 4. Launch workstation manager
```
roslaunch quadrotor_code bebop_manager.launch
```
Now you should be able to see the odom of each drone by Rviz. Note that it is important to set the number of drones and their initial position in the launch file. \
\* If some drones' odom are always at the origin, perhaps the odom messages are not recieved at all. Try something like `rostopic echo uav0/odom`.

## 5. Launch path planing node
```
roslaunch quadrotor_code path_plan.launch
```
## 6. Launch flocking algorithm nodes
Run the remote roslaunch file from the workstation.
```
roslaunch quadrotor_code act_six.launch
```
## 7. Take off
```
rosrun turtlebot_teleop swarmteleop
```
Then press v to let all drones take off and b to land. Note that the code supports as many as 9 drones with namespaces from uav0 to uav8. You may wish to change the code if you have more drones.  \
\* You can also press n to reset all drones but it is not recommanded. All drones will stop working in the air and drop down to the floor, which could cause serious damage to your drones or Raspberry PIs.

## 8. Set goal
Use the Rviz button `2D Nav Goal` and click on the map to set the goal. If the system works well, the planned path will appear.

## 9. Start flocking
Click the Rviz button `Publish Point` and then click on any point on the map. The flocking algorithm should be working now.

## 10. Land
Normally you can land all planes by pressing b in `swarmteleop`. Note that before pressing you shold select the terminal which runs `swarmteleop`.
