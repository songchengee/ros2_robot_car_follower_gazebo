from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node  # import Node
import yaml

urdf_path = '/home/song/ros2_ws/robot_car.urdf'
with open(urdf_path, 'r') as file:
    robot_description = file.read()

parameters=[{'robot_description': robot_description}]

robot_controllers = "/home/song/ros2_ws/src/follower/ros2_controller.yaml"



def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['sphere_controller', 'steering_controller'],
            output='screen'
        ),
        
        
    ])
