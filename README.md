# Sawyer\_catching_ball

###To run
####1.)

    roslaunch sawyer_catching_ball sawyer_sim.launch 

This line enables rviz with sawyer urdf and services.
Three services used are 

FwdKin : get joints position and return the end-effector transformation in /Pose

PoseToSO3 : get end-effector transformation in /Pose and return SE3 transformation

SO3ToPose : the inversion of PoseToSO3

!!! It shoud be SE3 not SO3, I will change that later.

####2.)

    rosrun sawyer_catching_ball sawyer_sim.py
This node publishes endpoint\_state and joint\_state, which is sent to rviz to move sawwyer, and acts as joint controller server who receives published /joint_command and move sawyer according to changed /joint\_state 

There as still an error in displaying the rotation of sawyer's display. I condier this tangential to fix at this point since it does not affect the position of the robot arm. 



####2.5) To verify the endpoint_state calculation, I already checked it.
    rosrun sawyer_catching_ball sawyer_joint_test.py
The node moves each joint of rviz around. We can do rostopic /robot/limb/right/endpoint_state and see if position of end-effeector endpoint is correct or not.

####3.) run the kinematic controller
    rosrun sawyer_catching_ball kin_cont_node_fix.py

This node runs the kinematic controller.
The calculation is based on end-effector frame.
Some functions are from robot\_calc\_functions.py by Mikhail from Kevin Lynch class.

The problems I suspect is that the [returned joint](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L128) from the [output](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L124) might be incorrect. Right now the [desired pose](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L260) is -0.0155 in x_axis from [home\_position](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L264) and [error](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L62) returned it correctly with no error in [w\_bd](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L60) and error in [v\_bd](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L61) as 0.0155. However, the printed joint velocity seems to be off with 
   
    [  1.20793724e-03   1.88287037e-18   1.61641281e-19   3.22099541e-19
      -6.34132277e-19  -1.23867129e-18  -1.84821351e-20] 
from the 
      
    [joint\_0 ... joint\_6] 
for the first calculation and laters. I think the first joint velocity seems to be obviously higher than the others and this seems weird to move the end-effector back in x-axis based on fixed base frame. I assume at this point that Mikhail's code is correct because we use [this line](https://github.com/ctanakul/sawyer-catching-ball/blob/master/src/kin_cont_node_fix.py#L128) in last quarter too.