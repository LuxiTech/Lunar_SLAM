import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _calibration_file(driver_share):
    configured = os.environ.get('LUXI_USB_CALIBRATION_FILE', '').strip()
    if configured:
        return os.path.expanduser(configured)
    workspace = os.environ.get('LUXI_WORKSPACE_ROOT', '').strip()
    if workspace:
        return os.path.join(
            os.path.expanduser(workspace), 'device', 'USBCameraSDK', 'ros2_ws',
            'calibration', 'stereo_opencv.yaml')
    return os.path.join(driver_share, 'config', 'stereo_opencv.yaml')


def generate_launch_description():
    driver_share = get_package_share_directory('usb_camera_driver')
    default_params = os.path.join(
        driver_share,
        'config',
        'stereo_calibration.yaml',
    )
    default_calibrator_params = os.path.join(
        driver_share,
        'config',
        'opencv_stereo_calibrator.yaml',
    )
    calibration_file = _calibration_file(driver_share)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'calibrator_params_file', default_value=default_calibrator_params),
        DeclareLaunchArgument('calibration_file', default_value=calibration_file),
        DeclareLaunchArgument('board_cols', default_value='11'),
        DeclareLaunchArgument('board_rows', default_value='8'),
        DeclareLaunchArgument('square_size', default_value='0.010'),
        Node(
            package='usb_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        TimerAction(
            period=1.0,
            actions=[Node(
                package='usb_camera_driver',
                executable='opencv_stereo_calibrator',
                name='opencv_stereo_calibrator',
                output='screen',
                parameters=[
                    LaunchConfiguration('calibrator_params_file'),
                    {
                        'output_file': LaunchConfiguration('calibration_file'),
                        'board_cols': ParameterValue(
                            LaunchConfiguration('board_cols'), value_type=int),
                        'board_rows': ParameterValue(
                            LaunchConfiguration('board_rows'), value_type=int),
                        'square_size': ParameterValue(
                            LaunchConfiguration('square_size'), value_type=float),
                    },
                ],
            )],
        ),
    ])
