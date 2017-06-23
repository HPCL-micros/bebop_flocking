1. roslaunch turtlebot_stage turtlebot_in_stage.launch
2. rostopic pub /uav0/move_base_simple/goal decide_softbus_msgs/NavigationPoint "
   header:
   seq: 0
   stamp: {secs: 0, nsecs: 0}
   frame_id: 'robot_0/base_link'
   pose:
   position: {x: 3.0, y: 3.0, z: 0.0}
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   arrival_time: {secs: 0, nsecs: 0}
   formation: 1
   parameters: [0]"
3. rosservice call /uav0/move_base/set_controlling "CONTROLLING: 1"
