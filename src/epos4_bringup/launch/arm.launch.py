# ===========================================================================
# Brings up EPOS4 axes under ros2_control.
#
#   ros2 launch epos4_bringup arm.launch.py                        # epos4_sim
#   ros2 launch epos4_bringup arm.launch.py can_interface:=can0    # hardware
#
# The simulator and the hardware use the SAME network, epos4_network:
# epos4_sim answers with the maxon identity, so only the interface changes.
# sim_network is for the canopen_fake_slaves mock; pointed at epos4_sim its
# boot aborts with es='D' before the PDO mapping is downloaded, and the
# master then decodes default-mapped frames as if they were its own.
#
# The DCF is taken from the installed share directory, where generate_dcf()
# puts it at build time, and passed as an absolute path: Lely resolves the
# UploadFile entries inside it at boot, not at load.
# ===========================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('epos4_bringup')

    can_interface = LaunchConfiguration('can_interface')
    master_dcf = LaunchConfiguration('master_dcf')
    description = LaunchConfiguration('description')

    # value_type=str: without it launch_ros tries to parse the generated XML
    # as YAML and fails on the first colon it finds.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', description,
                ' master_dcf:=', master_dcf,
                ' can_interface:=', can_interface,
            ]),
            value_type=str)
    }

    controllers = os.path.join(share, 'config', 'controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # The broadcaster first, the trajectory controller only once it is up:
    # spawning both at once races the hardware activation, and a controller
    # that claims its interfaces before they exist is rejected.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value='vcan0',
            description='SocketCAN interface: vcan0 for epos4_sim, can0 for hardware'),
        DeclareLaunchArgument(
            'master_dcf',
            default_value=os.path.join(share, 'config', 'epos4_network', 'master.dcf'),
            description='Absolute path to the master DCF generated from bus.yml'),
        DeclareLaunchArgument(
            'description',
            default_value=os.path.join(share, 'config', 'epos4_arm.urdf.xacro'),
            description='Robot description (xacro) that calls epos4_system'),

        control_node,
        state_publisher,
        jsb_spawner,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])),
    ])
