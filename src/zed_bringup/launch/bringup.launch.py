from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_mapping = LaunchConfiguration('enable_mapping')

    return LaunchDescription([
        DeclareLaunchArgument('enable_mapping', default_value='true',
                               description='매핑 노드도 같이 켤지 여부'),

        # --- zed_perception ---
        Node(package='zed_perception', executable='camera_node',
             name='camera_node', output='screen'),
        Node(package='zed_perception', executable='perception_bridge_node',
             name='perception_bridge_node', output='screen'),
        Node(package='zed_perception', executable='mapping_node',
             name='mapping_node', output='screen',
             condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(enable_mapping)),

        # --- zed_mission ---
        Node(package='zed_mission', executable='path_planner_node',
             name='path_planner_node', output='screen'),
        Node(package='zed_mission', executable='safety_supervisor_node',
             name='safety_supervisor_node', output='screen'),
        Node(package='zed_mission', executable='helm_node',
             name='helm_node', output='screen'),
        Node(package='zed_mission', executable='mission_manager_node',
             name='mission_manager_node', output='screen'),
        Node(package='zed_mission', executable='thruster_output',
             name='thruster_output', output='screen'),
        Node(package='zed_mission', executable='dashboard_node',
             name='dashboard_node', output='screen'),
    ])
