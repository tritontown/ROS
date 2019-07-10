# ROS
this is apriltag detection packaged within ROS and visualizable in rviz.


to run, dl files into catkin_ws/apriltag on a system with ROS.

next, run $ roscore.

then, $ ./april_tag.py

then, $ ./rvizSim.py

then, $ rosrun rviz rviz

inside rviz, add a marker. change the marker's topic to /marker_redball.

inside rviz, change "fixed frame" to /base_link.

on the right side of rviz, change camera to topdown.

run $ rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50

now, you should be able to see the box moving and rotating with your tag.



i will go through and clean up the code soon and hopefully automatically initialize rviz, it's a mess right now.
