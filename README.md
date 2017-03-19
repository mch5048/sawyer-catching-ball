# Sawyer catch a ball

### Goal
To create algorithm and implement it on Sawyer, the manufacturing robot by Rethink Robotics, to
catch a ball thrown at him

### Hardware
Sawyer and stereo camera with RGB and depth

![alt tag](https://github.com/ctanakul/sawyer-catching-ball/blob/master/etc/pict/readme_pict_1.png)

### Ball detection system

The ball detection system is comprised of image processing with color detection and 3D position
finding from pointcloud.

The hardware facilitating in color detection and 3D position finding is [Asus Xtion Pro](https://www.asus.com/us/3D-Sensor/Xtion_PRO_LIVE/) Live with RGB
and depth sensor. The software for connecting and processing information from the hardware is
[openni2](http://wiki.ros.org/openni2_launch) package.

To get the 3D position of the ball, the image processing first detects the position of the ball in RGB
image with color detection algorithm. The image is smoothened with GaussianBlur and converted into
HSV image to extract only a portion within specified HSV range. Noise is eroded and the left portion
is dilated. The unmasked portion is further enclosed by a circle whose center pixel is outputted into
[projectPixelTo3dRay](http://docs.ros.org/api/image_geometry/html/python/#image_geometry.PinholeCameraModel.projectPixelTo3dRay) to get the unit vector pointing to 3D point from registered pointcloud With the
unit vector and depth data of each pixel from depth image, the 3D position of the ball can be found.
Users can adjust the HSV range and the size of eroding and dilating kernel with trackbars provided
with the algorithm.

### Projectile calculating algorithm
The projectile calculating algorithm estimates the future position of the ball from the first few
positions in 3D at the beginning of the throw. Every two positions can be used to find initial velocity,
launch angle in horizontal and vertical plane for estimating the future position of the ball.

In order to check the precision of the system without running real robot, the predicting position is
simulated as a frame in RVIZ to compare between the last position of the ball in physical and
simulated world.

### Inverse Kinematics Controller
The frequency of published pointcloud is adjusted to be at 30 Hz. The assumed number of 3D
positions needed for future position estimating is 5 positions which takes about 15 milliseconds. Since
the estimated time used in a throw is about 1 second, the time for the arm to move to catch the ball is
about 85 milliseconds. For simplicity, the planar area for moving hand is specified. This area can be
found by moving the arm and timing the total time used.

Before every move, the hand will be moved to home position by sending the dictionary containing
names and joint positions as a parameter in [move\_to\_joint\_positions](http://api.rethinkrobotics.com/baxter_interface/html/baxter_interface.limb.Limb-class.html) function of class Limb. The
[damped least square inverse kinematics](http://web.cse.ohio-state.edu/~parent/classes/694A/Lectures/Material/IKsurvey.pdf) is used to find joint velocity for each joint in robot arm while
moving. The damping constant delays time in finding joint solution to avoid singularities. However,
high damping constant will slow the convergence time a lot. Joint angular velocities are then
published under the topic of robot/limb/right/joint\_command of type
[intera\_core\_msgs/JointCommand](http://api.rethinkrobotics.com/baxter_core_msgs/html/msg/JointCommand.html).

The area that the hand can be moved within 85 milliseconds is approximately 30 x 30 cm square on
the right side and semicircle with 30 cm radius on the left side of the home position.