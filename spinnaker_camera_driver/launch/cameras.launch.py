from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml

parameters = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': False,
    'dump_node_map': False,
    'ir_frame_rate': 'Rate30Hz',
    'pixel_format': 'Mono8',
    'gev_scps_packet_size': 1444,
    'image_width': 640,
    'image_height': 480,
    'offset_x': 0,
    'offset_y': 0,
    'nuc_mode': 'Automatic',
    'frame_rate': 15.0,
}


def load_cameras(context):
    """Load camera parameters from a YAML file."""
    yaml_file = LaunchConfig('camera_config_file').perform(context)
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return data['cameras']


def make_camera_node(name, camera_type, serial):
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml']
    )

    node = ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=name,
        parameters=[parameters, {'parameter_file': parameter_file, 'serial_number': serial}],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    return node


def launch_setup(context, *args, **kwargs):
    """Create multiple camera nodes."""
    cameras = load_cameras(context)
    composable_nodes = [
        make_camera_node(
            cam['name'],
            cam['type'],
            cam['serial']
        ) for cam in cameras
    ]
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'camera_config_file',
                default_value='/home/mess2/Desktop/config/flir.yaml',
                description='Path to the YAML file containing camera configurations.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
