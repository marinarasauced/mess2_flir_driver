
# LAUNCH THIS FILE WITH THE FOLLOWING COMMAND:
# ros2 launch mess2_flir_driver camera.launch.py serial:='"SERIAL"'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

parameters = {
    'flir_a70': {
        'debug': False,
        'compute_brightness': False,
        'adjust_timestamp': False,
        'dump_node_map': False,
        # --- 
        'pixel_format': 'Mono8', #YUV422_8_UYVY
        'gev_scps_packet_size': 1444,
        'image_width': 640,
        'image_height': 480,
        'offset_x': 0,
        'offset_y': 0,
        'nuc_mode': 'Automatic',
        'image_adjust_method': 'Auto',
        'video_orientation': 'Normal',
    },
}


def launch_setup(context, *args, **kwargs):
    """
    
    """
    parameter_file = LaunchConfig('parameter_file').perform(context)
    camera_type = LaunchConfig('camera_type').perform(context)
    if not parameter_file:
        parameter_file = PathJoinSubstitution(
            [FindPackageShare('mess2_flir_launch'), 'config/config', camera_type + '.yaml']
    )
    if camera_type not in parameters:
        raise Exception('no example parameters available for type ' + camera_type)

    node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name=[LaunchConfig('camera_name')],
        parameters=[
            parameters[camera_type],
            {
                'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                'parameter_file': parameter_file,
                'serial_number': [LaunchConfig('serial')],
            },
        ],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ],
    )

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'camera_name',
                default_value=['flir_camera'],
                description='camera name (ros node name)',
            ),
            LaunchArg(
                'camera_type',
                default_value='flir_a70',
                description='type of camera (blackfly_s, chameleon...)',
            ),
            LaunchArg(
                'serial',
                default_value="'01234567'",
                description='FLIR serial number of camera (in quotes!!)',
            ),
            LaunchArg(
                'parameter_file',
                default_value='',
                description='path to ros parameter definition file (override camera type)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )