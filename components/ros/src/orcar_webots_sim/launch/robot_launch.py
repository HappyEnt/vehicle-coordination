import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

def is_docker():
   return os.path.exists('/.dockerenv')
    # return false

def generate_launch_description():
    package_dir = get_package_share_directory('orcar_webots_sim')
    orcar_description  = pathlib.Path(os.path.join(package_dir, 'resource', 'orcar.urdf')).read_text()
    anchor_description = pathlib.Path(os.path.join(package_dir, 'resource', 'anchor.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'orcar_world.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    controller_url = 'tcp://' + 'host.docker.internal' + ':1234/' if is_docker() else ''

    print('connecting to controller at: ' + controller_url)

    orcar_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url + 'orcar'},
        parameters=[
            {'robot_description': orcar_description},
            {'particles': 500},
        ]
    )

    anchor_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url + 'anchor'},
        parameters=[
            {'robot_description': anchor_description},
            {'frequency': 10}, # 10 hz beacon send rate
        ]
    )


    localization = Node(
        executable='belief_propagation',
        package='orcar_localization',
        namespace="orcar",
        parameters=[
          {"receive_queue_topicname": "/orcar/ranging_radio/receive_queue"}
        ],
        output='screen'
    )

    return LaunchDescription([
        webots,
        orcar_driver,
        anchor_driver,
        ros2_supervisor,
        localization,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
