# evs

This package provides a ROS node that generate an assumed
reference camera's trajectory based on one obtained using a visual servoing scheme.
The resulting data can be then handled, evaluated and compared using the 
evo tools (https://github.com/MichaelGrupp/evo).

##### Expected input data:
A ROS bag that contain, at least, two "geometry_msgs/PoseStamped" topics, 
one for the camera pose at each iteration of the visual servoing, and one 
for the desired camera pose 
(see the sample code below to create a ROS bag in cpp).

```cpp
rosbag::Bag vsbag;
vsbag.open("vs.bag",rosbag::bagmode::Write);
[VS INIT]
vsbag.write("/vs/camera/desired_pose", ros::Time::now(), desiredRobotPoseStamped);
[VS MAIN LOOP START]
    vsbag.write("/vs/camera/current_pose", ros::Time::now(), currentRobotPoseStamped);
[VS MAIN LOOP END]
vsbag.close();
```

##### Usage:
- Edit the launch file parameters: 
    - pathToInputBag:   path to find the VS ROS bag (see the sample code above) 
    - desiredPoseTopic: name of the desired pose topic in the input ROS bag (e.g., "/vs/camera/desired_pose")
    - currentPoseTopic: name of the current poses topic in the input ROS bag (e.g., "/vs/camera/current_pose")
    - pathToOutputBag:  path to save the generated ROS bag data usable with evo
- Then, ```roslaunch  generateVSReferenceTrajectory.launch```

The resulting ROS bag can be used with evo tools:
- ```evo_traj bag output1.bag /vs/reference /vs/current -p```
- ```evo_ape bag output1.bag /vs/reference /vs/current  --save_results output1.ape```
- ```evo_ape bag output2.bag /vs/reference /vs/current  --save_results output2.ape```
- ```evo_res output1.ape output2.ape -p --use_filenames```
- etc... (for more, see https://github.com/MichaelGrupp/evo)

