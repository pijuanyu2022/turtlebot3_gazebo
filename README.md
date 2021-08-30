# Use turtlebot3 to follow a yellow line

Step1: git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

Step2: git clone https://github.com/pijuanyu2022/turtlebot3_gazebo.git

Step3: catkin_make

Step4:export TURTLEBOT3_MODEL=waffle pi

Step5: roslaunch turtlebot3_gazebo turtlebot3_follower.launch

Step6: rosrun turtlebot3_gazebo follower.py

---

SLAM:

Step1: export TURTLEBOT3_MODEL = burger

Step2: roslaunch turtlebot3_gazebo turtlebot3_world.launch

Step3: roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

Step4: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
