## CHEATS:

###  PUBLISH GOAL
```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: 1.0, y: 0.0, z: 0.0}}"
```

### PUBLISH POINT
```bash
ros2 topic pub -1 /g1pilot/left_hand_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'pelvis'}, pose: {position: {x: 0.40, y: 0.17, z: 0.09}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### PUBLISH JOY
```bash
ros2 topic pub --once /g1pilot/joy sensor_msgs/msg/Joy '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, axes: [0,0,0,0,0,0,0,0], buttons: [1,0,0,0,0,0,0,0,0,0,0,0]}'
```

### CONTROL DX3 HAND (for left and right hand)
```bash
ros2 topic pub --once /g1pilot/dx3/right/hand_action std_msgs/msg/String "{data: 'close'}"
```
```bash
ros2 topic pub --once /g1pilot/dx3/left/hand_action std_msgs/msg/String "{data: 'open'}"
```

### STANDARD SEQUENCE FOR UNITREE ROBOT
```bash
1.- Put the battery and turn on the robot
```
```bash
2.- ros2 launch g1pilot robot_state_launcher.launch.py use_robot:=True publish_joint_states:=True interface:=eth0
```
```bash
3.- ros2 launch g1pilot navigation_launcher.launch.py use_robot:=True interface:=eth0 arm_controlled:='both' enable_arm_ui:=True ik_use_waist:=False
```
```bash
4.- ros2 run g1pilot interactive_marker
```