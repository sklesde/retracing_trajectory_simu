import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='trajectory_tools',
            executable='trajectory_recorder',
            name='trajectory_recorder',
        #     remappings=[
        #     ("/tf","/cortex/tf"),
        #     ("/tf_static","/cortex/tf_static")
        # ],
        ),
        Node(
            package='trajectory_tools',
            executable='trajectory_retracing',
            name='trajectory_retracing',
        #     remappings=[
        #     ("/tf","/cortex/tf"),
        #     ("/tf_static","/cortex/tf_static")
        # ],
        ),
        Node(
            package='trajectory_tools',
            executable='trajectory_smoother',
            name='trajectory_smoother',
        #     remappings=[
        #     ("/tf","/cortex/tf"),
        #     ("/tf_static","/cortex/tf_static")
        # ],
        ),
        Node(
            package='trajectory_logger',
            executable='trajectory_logger_node',
            name='trajectory_logger',
        #     remappings=[
        #     ("/tf","/cortex/tf"),
        #     ("/tf_static","/cortex/tf_static")
        # ],
        )
    ])