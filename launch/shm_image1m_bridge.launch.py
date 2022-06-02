import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package='shm_msgs', executable='shm_image1m_bridge', output='screen',
            remappings=[
                ('shm_image_input', 'shm_image_1m'),
                ('sensor_image_output', 'sensor_image_1m')
            ],
            parameters=[
            ]
        ),
    ])
