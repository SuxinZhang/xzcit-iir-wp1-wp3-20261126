"""启动文件：摄像头 + 激光雷达 + 融合 + 网页调试。

第一阶段约束：enable_cmd_vel 默认 false，不向底盘发速度指令。

设备路径默认值来自香橙派现场实测（2026-11-06 只读核对）：

- 雷达：``/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0``
  by-path 含 USB 控制器与物理端口号，换 USB 口就会变。
- 摄像头：``/dev/v4l/by-id/usb-HD_720P_Webcam_HD_720P_Webcam_20130901-video-index0``
  by-id 含设备序列号，插拔或重新枚举后编号变化也不受影响。

不要再写 /dev/video0 或 /dev/ttyUSB0：枚举顺序变化会指向别的设备。
两者都可用 launch 参数覆盖，详见 docs/device-paths.md。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成四个节点的启动描述。"""
    video_device = LaunchConfiguration('video_device')
    lidar_port = LaunchConfiguration('lidar_port')
    enable_cmd = LaunchConfiguration('enable_cmd_vel')
    angle_origin = LaunchConfiguration('angle_origin')
    patrol_map_yaml = LaunchConfiguration('patrol_map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=('/dev/v4l/by-id/'
                           'usb-HD_720P_Webcam_HD_720P_Webcam_20130901'
                           '-video-index0'),
            description='摄像头设备节点，用 /dev/v4l/by-id 稳定路径'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=('/dev/serial/by-path/'
                           'platform-fc8c0000.usb-usb-0:1:1.0-port0'),
            description='激光雷达串口，必须是现场实测的 by-path 稳定路径'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false',
                              description='是否发布 cmd_vel，第一阶段保持 false'),
        DeclareLaunchArgument('angle_origin', default_value='mirror',
                              description='角度约定：mirror 或 raw'),
        # 巡航路线绑定地图内容（yaml 内 image 指向的 pgm 一并哈希）。
        # 不给就保持空串：网页侧按"地图未知"拒绝保存/开始巡航，
        # 不会拿一条来历不明的路线去跑。
        DeclareLaunchArgument(
            'patrol_map_yaml', default_value='',
            description='地图 yaml 路径（SLAM/nav2 保存的地图），路线按它绑定'),
        DeclareLaunchArgument('enable_navigation_vision', default_value='false'),
        DeclareLaunchArgument('vision_model_file', default_value=''),
        DeclareLaunchArgument('camera_calibration_file', default_value=''),
        DeclareLaunchArgument('ground_calibration_file', default_value=''),
        DeclareLaunchArgument('body_half_length', default_value='0.0'),
        DeclareLaunchArgument('body_half_width', default_value='0.0'),
        DeclareLaunchArgument('body_center_x', default_value='0.0'),
        DeclareLaunchArgument('charging_config', default_value=str(
            Path(__file__).resolve().parents[1] / 'config' / 'wp3_charging.yaml')),
        DeclareLaunchArgument('enable_camera_tf', default_value='false'),
        DeclareLaunchArgument('camera_x', default_value=''),
        DeclareLaunchArgument('camera_y', default_value=''),
        DeclareLaunchArgument('camera_z', default_value=''),
        DeclareLaunchArgument('camera_roll', default_value=''),
        DeclareLaunchArgument('camera_pitch', default_value=''),
        DeclareLaunchArgument('camera_yaw', default_value=''),
        # Supply measured camera mounting coordinates before enabling this TF.
        # 启用此 TF 前必须提供实测相机安装坐标。
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_mount_tf',
             condition=IfCondition(LaunchConfiguration('enable_camera_tf')),
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'camera_link',
                        '--x', LaunchConfiguration('camera_x'),
                        '--y', LaunchConfiguration('camera_y'),
                        '--z', LaunchConfiguration('camera_z'),
                        '--roll', LaunchConfiguration('camera_roll'),
                        '--pitch', LaunchConfiguration('camera_pitch'),
                        '--yaw', LaunchConfiguration('camera_yaw')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_optical_tf',
             condition=IfCondition(LaunchConfiguration('enable_camera_tf')),
             arguments=['--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame',
                        '--x', '0', '--y', '0', '--z', '0',
                        '--roll', '-1.5707963267948966', '--pitch', '0',
                        '--yaw', '-1.5707963267948966']),

        # Consume the existing camera stream; never open another camera device.
        # 订阅现有相机图像流，禁止打开第二个相机设备。
        Node(package='iir_navigation_vision', executable='vision_node',
             name='iir_navigation_vision', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_navigation_vision')),
             parameters=[{
                 'model_file': LaunchConfiguration('vision_model_file'),
                 'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
                 'ground_calibration_file': LaunchConfiguration('ground_calibration_file'),
             }]),


        Node(
            package='opi_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device': video_device,
                'width': 1280,
                'height': 720,
                'pixel_format': 'MJPG',
                'frame_id': 'camera_link',
            }],
        ),

        Node(
            package='opi_robot',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'lidar_port': lidar_port,
                'baudrate': 230400,
                'frame_id': 'laser',
                'bins': 360,
                'range_min': 0.02,
                'range_max': 12.0,
                'stale_point_age': 0.5,
                'origin': angle_origin,
                'angle_offset_deg': 0.0,
                'frame_timeout': 0.5,
                'recovery_coverage': 0.9,
                'reconnect_interval': 2.0,
            }],
        ),

        Node(
            package='opi_robot',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                # 停车距离取 0.4 m：2026-11-14 用户定（原 0.25 m）。
                'stop_distance': 0.4,
                'slow_distance': 0.90,
                'front_half_angle_deg': 30.0,
                # 与底盘桥 PLACEHOLDER_MAX_LINEAR_MPS 保持一致，超限会被判 NAV_INVALID
                'max_linear': 0.4,
                'enable_cmd_vel': enable_cmd,
                'cruise_timeout': 0.0,   # ≤0 = 无限运行直到手动停止
                # 巡航档 0.4 m/s；融合节点取它与 max_linear 的较小值。
                'cruise_linear': 0.4,
                'scan_timeout': 1.0,
                'lidar_diag_timeout': 2.0,
                'auto_turn': False,
                'body_half_length': ParameterValue(LaunchConfiguration('body_half_length'), value_type=float),
                'body_half_width': ParameterValue(LaunchConfiguration('body_half_width'), value_type=float),
                'body_center_x': ParameterValue(LaunchConfiguration('body_center_x'), value_type=float),
            }],
        ),

        Node(
            package='opi_robot',
            executable='web_debug',
            name='web_debug',
            output='screen',
            # 控制台显示的安全开关。launch 里如实传 false/false：
            # 这里只负责"显示真实值"，不负责打开驱动。
            parameters=[LaunchConfiguration('charging_config'), {
                'drive_enable': False,
                'drive_input_enable': False,
                'drive_firmware_protocol': 'baseline',
                'patrol_map_yaml': patrol_map_yaml,
            }],
        ),
    ])
