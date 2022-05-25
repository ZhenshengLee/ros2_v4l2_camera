import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():

    return launch.LaunchDescription([
        Trace(
            session_name='v4l2_trace',
            events_kernel=[]
        ),
       launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen',
            # remappings=[
            #     ('input1', 'topic1'),
            #     ('input2', 'topic2')
            # ],
            # parameters=[
            #     {'period1_ns': period1_ns},
            #     {'period2_ns': period2_ns},
            #     {'callback_duration_ns': callback_duration_ns}
            # ]
        ),

        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_image_subscriber', output='screen',
            # remappings=[
            #     ('input', 'topic1'),
            #     ('output', 'topic3'),
            # ],
            # parameters=[
            #     {'callback_duration_ns': callback_duration_ns}
            # ]
        ),
    ])
