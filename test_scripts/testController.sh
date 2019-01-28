rostopic pub -1 /orbot/space/vicon geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: 
    secs: 1
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: -100.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

rostopic pub -1 /orbot/space/state/truth nearlab_msgs/StateStamped "header:
  seq: 0
  stamp: {secs: 1, nsecs: 0}
  frame_id: ''
v: {x: 0.0, y: 0.0, z: 0.0}
w: {x: 0.0, y: 0.0, z: 0.0}
r: {x: -100.0, y: 0.0, z: 0.0}
q: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" 

