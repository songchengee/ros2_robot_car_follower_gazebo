# ros2_robot_car_follower_gazebo
A follower robotcar that is built from scratch, including the follower algorithm, URDF, and controller set up. It is simulated in Gazebo, and using ROS2 as the controller.

Steps to run it:
- Convert Xacro to URDF
- Luanch follower gazebo.launch.py (this will start Gazebo, and spawn the controllers)
- Spawn the car with the command: ros2 run gazebo_ros spawn_entity.py   -file robot_car.urdf   -entity robot_car   -x 0 -y 0 -z 0.1
- Run the follower script (will subscribe camera depth data, process them, going through algorithm and publish instructions to the robot car to follow the obstacle)
- Insert an obstacle infront of the car

I build it from scratch instead of using ros2/gazebo tutorial plugin (that already done everything for you) just for learning purpose.
