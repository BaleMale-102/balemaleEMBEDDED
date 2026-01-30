# rc_bringup/launch/sensors.launch.py
# 수정: IfCondition 조건문 버그 fix (PythonExpression 사용)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _bool(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'on')


def _guard_exclusive(context, *args, **kwargs):
    real = _bool(context, 'use_real_cameras')
    if not real:
        return [LogInfo(msg='[WARN] use_real_cameras=false. No /cam_*/image_raw will be published.')]
    return []


def _v4l2_set_front_camera(context, *args, **kwargs):
    if (not _bool(context, 'use_real_cameras')) or (not _bool(context, 'use_v4l2_controls')) or (not _bool(context, 'use_front_camera')):
        return []

    front_dev = LaunchConfiguration('front_by_path').perform(context)

    ae = LaunchConfiguration('front_auto_exposure').perform(context)
    exp = LaunchConfiguration('front_exposure').perform(context)
    gain = LaunchConfiguration('front_gain').perform(context)
    bri = LaunchConfiguration('front_brightness').perform(context)

    cmd = (
        f"v4l2-ctl -d {front_dev} -c auto_exposure={ae} && "
        f"v4l2-ctl -d {front_dev} -c exposure_time_absolute={exp} && "
        f"v4l2-ctl -d {front_dev} -c gain={gain} && "
        f"v4l2-ctl -d {front_dev} -c brightness={bri} && "
        f"v4l2-ctl -d {front_dev} --all | egrep 'auto_exposure|exposure_time_absolute|gain|brightness' || true"
    )

    return [
        LogInfo(msg=f"[v4l2] applying FRONT camera controls on {front_dev}"),
        ExecuteProcess(cmd=['bash', '-lc', cmd], output='screen')
    ]


def generate_launch_description():
    # toggles
    use_real_cameras = LaunchConfiguration('use_real_cameras')
    use_static_tf = LaunchConfiguration('use_static_tf')
    use_imu = LaunchConfiguration('use_imu')

    use_front_camera = LaunchConfiguration('use_front_camera')
    use_bottom_camera = LaunchConfiguration('use_bottom_camera')
    use_side_camera = LaunchConfiguration('use_side_camera')

    # camera args
    usb_fps = LaunchConfiguration('usb_fps')
    usb_w = LaunchConfiguration('usb_width')
    usb_h = LaunchConfiguration('usb_height')
    usb_pixfmt = LaunchConfiguration('usb_pixel_format')
    usb_io = LaunchConfiguration('usb_io_method')

    bottom_by = LaunchConfiguration('bottom_by_path')
    front_by = LaunchConfiguration('front_by_path')
    side_by = LaunchConfiguration('side_by_path')

    # frames
    base_frame = LaunchConfiguration('tf_base_frame')
    bottom_frame = LaunchConfiguration('tf_bottom_frame')
    front_frame = LaunchConfiguration('tf_front_frame')
    side_frame = LaunchConfiguration('tf_side_frame')

    imu_frame = LaunchConfiguration('imu_frame_id')

    # static TF params (cameras)
    bottom_x = LaunchConfiguration('tf_bottom_x'); bottom_y = LaunchConfiguration('tf_bottom_y'); bottom_z = LaunchConfiguration('tf_bottom_z')
    bottom_roll = LaunchConfiguration('tf_bottom_roll'); bottom_pitch = LaunchConfiguration('tf_bottom_pitch'); bottom_yaw = LaunchConfiguration('tf_bottom_yaw')

    front_x = LaunchConfiguration('tf_front_x'); front_y = LaunchConfiguration('tf_front_y'); front_z = LaunchConfiguration('tf_front_z')
    front_roll = LaunchConfiguration('tf_front_roll'); front_pitch = LaunchConfiguration('tf_front_pitch'); front_yaw = LaunchConfiguration('tf_front_yaw')

    side_x = LaunchConfiguration('tf_side_x'); side_y = LaunchConfiguration('tf_side_y'); side_z = LaunchConfiguration('tf_side_z')
    side_roll = LaunchConfiguration('tf_side_roll'); side_pitch = LaunchConfiguration('tf_side_pitch'); side_yaw = LaunchConfiguration('tf_side_yaw')

    # static TF params (imu)
    imu_x = LaunchConfiguration('tf_imu_x'); imu_y = LaunchConfiguration('tf_imu_y'); imu_z = LaunchConfiguration('tf_imu_z')
    imu_roll = LaunchConfiguration('tf_imu_roll'); imu_pitch = LaunchConfiguration('tf_imu_pitch'); imu_yaw = LaunchConfiguration('tf_imu_yaw')

    # ===== 🔧 FIX: PythonExpression으로 조건 결합 =====
    cond_front = IfCondition(PythonExpression([
        "'", use_real_cameras, "'=='true' and '", use_front_camera, "'=='true'"
    ]))
    cond_bottom = IfCondition(PythonExpression([
        "'", use_real_cameras, "'=='true' and '", use_bottom_camera, "'=='true'"
    ]))
    cond_side = IfCondition(PythonExpression([
        "'", use_real_cameras, "'=='true' and '", use_side_camera, "'=='true'"
    ]))

    cond_tf_front = IfCondition(PythonExpression([
        "'", use_static_tf, "'=='true' and '", use_front_camera, "'=='true'"
    ]))
    cond_tf_bottom = IfCondition(PythonExpression([
        "'", use_static_tf, "'=='true' and '", use_bottom_camera, "'=='true'"
    ]))
    cond_tf_side = IfCondition(PythonExpression([
        "'", use_static_tf, "'=='true' and '", use_side_camera, "'=='true'"
    ]))

    # ===== Cameras =====
    cam_bottom = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='cam_bottom', output='screen',
        parameters=[{
            'video_device': bottom_by,
            'framerate': usb_fps, 'image_width': usb_w, 'image_height': usb_h,
            'pixel_format': usb_pixfmt, 'io_method': usb_io,
            'frame_id': bottom_frame,
            # 자동설정 비활성화 (unknown control 에러 방지)
            'auto_white_balance': False,
            'autoexposure': False,
            'autofocus': False,
        }],
        remappings=[('image_raw', '/cam_bottom/image_raw'), ('camera_info', '/cam_bottom/camera_info')],
        condition=cond_bottom,
    )

    cam_front = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='cam_front', output='screen',
        parameters=[{
            'video_device': front_by,
            'framerate': usb_fps, 'image_width': usb_w, 'image_height': usb_h,
            'pixel_format': usb_pixfmt, 'io_method': usb_io,
            'frame_id': front_frame,
            'camera_info_url': 'file:///home/a102/ws/config/calibration_front/ost.yaml',
            # Brio 100: 고정포커스, 컨트롤명 다름 → 자동설정 비활성화
            'auto_white_balance': False,
            'autoexposure': False,
            'autofocus': False,
            'brightness': 128,
            'contrast': 128,
            'saturation': 128,
            'sharpness': 128,
            'gain': 10,
            'exposure': 100,
        }],
        remappings=[('image_raw', '/cam_front/image_raw'), ('camera_info', '/cam_front/camera_info')],
        condition=cond_front,
    )

    cam_side = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='cam_side', output='screen',
        parameters=[{
            'video_device': side_by,
            'framerate': usb_fps, 'image_width': usb_w, 'image_height': usb_h,
            'pixel_format': usb_pixfmt, 'io_method': usb_io,
            'frame_id': side_frame,
            # 자동설정 비활성화
            'auto_white_balance': False,
            'autoexposure': False,
            'autofocus': False,
        }],
        remappings=[('image_raw', '/cam_side/image_raw'), ('camera_info', '/cam_side/camera_info')],
        condition=cond_side,
    )

    # ===== Static TF (cameras) =====
    static_tf_bottom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_bottom', output='screen',
        arguments=[bottom_x, bottom_y, bottom_z, bottom_roll, bottom_pitch, bottom_yaw, base_frame, bottom_frame],
        condition=cond_tf_bottom,
    )

    static_tf_front = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_front', output='screen',
        arguments=[front_x, front_y, front_z, front_roll, front_pitch, front_yaw, base_frame, front_frame],
        condition=cond_tf_front,
    )

    static_tf_side = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_side', output='screen',
        arguments=[side_x, side_y, side_z, side_roll, side_pitch, side_yaw, base_frame, side_frame],
        condition=cond_tf_side,
    )

    # ===== IMU Node =====
    imu_node = Node(
        package='rc_sensors',
        executable='mpu6050_imu_node',
        name='mpu6050_imu_node',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
            'i2c_addr': LaunchConfiguration('imu_i2c_addr'),
            'frame_id': imu_frame,
            'publish_rate_hz': LaunchConfiguration('imu_rate_hz'),

            'accel_range_g': LaunchConfiguration('imu_accel_range_g'),
            'gyro_range_dps': LaunchConfiguration('imu_gyro_range_dps'),
            'dlpf': LaunchConfiguration('imu_dlpf'),
            'sample_div': LaunchConfiguration('imu_sample_div'),

            'calib_seconds': LaunchConfiguration('imu_calib_seconds'),
            'use_complementary_filter': LaunchConfiguration('imu_use_cf'),
            'comp_alpha': LaunchConfiguration('imu_comp_alpha'),
        }],
        condition=IfCondition(use_imu),
    )

    # ===== Static TF (base_link -> imu_link) =====
    static_tf_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_imu', output='screen',
        arguments=[imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw, base_frame, imu_frame],
        condition=IfCondition(use_static_tf),
    )

    return LaunchDescription([
        # toggles
        DeclareLaunchArgument('use_real_cameras', default_value='true'),
        DeclareLaunchArgument('use_static_tf', default_value='true'),
        DeclareLaunchArgument('use_v4l2_controls', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),

        # per-camera enable
        DeclareLaunchArgument('use_front_camera', default_value='true'),
        DeclareLaunchArgument('use_bottom_camera', default_value='true'),
        DeclareLaunchArgument('use_side_camera', default_value='false'),

        # usb cam params
        DeclareLaunchArgument('usb_fps', default_value='30.0'),
        DeclareLaunchArgument('usb_width', default_value='640'),
        DeclareLaunchArgument('usb_height', default_value='480'),
        DeclareLaunchArgument('usb_pixel_format', default_value='yuyv'),
        DeclareLaunchArgument('usb_io_method', default_value='mmap'),

        # v4l2 controls (front)
        DeclareLaunchArgument('front_auto_exposure', default_value='1'),
        DeclareLaunchArgument('front_exposure', default_value='100'),
        DeclareLaunchArgument('front_gain', default_value='10'),
        DeclareLaunchArgument('front_brightness', default_value='128'),

        # device paths
        DeclareLaunchArgument('bottom_by_path', default_value='/dev/video2'),
        DeclareLaunchArgument('front_by_path', default_value='/dev/video0'),
        DeclareLaunchArgument('side_by_path', default_value='/dev/video3'),

        # frames (cameras)
        DeclareLaunchArgument('tf_base_frame', default_value='base_link'),
        DeclareLaunchArgument('tf_bottom_frame', default_value='camera_bottom'),
        DeclareLaunchArgument('tf_front_frame', default_value='camera_front'),
        DeclareLaunchArgument('tf_side_frame', default_value='camera_side'),

        # IMU frame
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        # static TF bottom
        DeclareLaunchArgument('tf_bottom_x', default_value='0.16'),
        DeclareLaunchArgument('tf_bottom_y', default_value='0.0'),
        DeclareLaunchArgument('tf_bottom_z', default_value='-0.05'),
        DeclareLaunchArgument('tf_bottom_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_bottom_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_bottom_yaw', default_value='0.0'),

        # static TF front
        DeclareLaunchArgument('tf_front_x', default_value='0.06'),
        DeclareLaunchArgument('tf_front_y', default_value='0.0'),
        DeclareLaunchArgument('tf_front_z', default_value='0.03'),
        DeclareLaunchArgument('tf_front_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_front_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_front_yaw', default_value='0.0'),

        # static TF side
        DeclareLaunchArgument('tf_side_x', default_value='0.0'),
        DeclareLaunchArgument('tf_side_y', default_value='-0.08'),
        DeclareLaunchArgument('tf_side_z', default_value='-0.04'),
        DeclareLaunchArgument('tf_side_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_side_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_side_yaw', default_value='0.0'),

        # static TF imu
        DeclareLaunchArgument('tf_imu_x', default_value='0.0'),
        DeclareLaunchArgument('tf_imu_y', default_value='0.0'),
        DeclareLaunchArgument('tf_imu_z', default_value='0.0'),
        DeclareLaunchArgument('tf_imu_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('tf_imu_yaw', default_value='0.0'),

        # IMU params
        DeclareLaunchArgument('imu_i2c_bus', default_value='7'),
        DeclareLaunchArgument('imu_i2c_addr', default_value='104'),
        DeclareLaunchArgument('imu_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('imu_accel_range_g', default_value='2'),
        DeclareLaunchArgument('imu_gyro_range_dps', default_value='250'),
        DeclareLaunchArgument('imu_dlpf', default_value='3'),
        DeclareLaunchArgument('imu_sample_div', default_value='4'),
        DeclareLaunchArgument('imu_calib_seconds', default_value='2.0'),
        DeclareLaunchArgument('imu_use_cf', default_value='true'),
        DeclareLaunchArgument('imu_comp_alpha', default_value='0.98'),

        # guards + v4l2 apply
        OpaqueFunction(function=_guard_exclusive),
        OpaqueFunction(function=_v4l2_set_front_camera),

        # nodes
        cam_bottom, cam_front, cam_side,
        static_tf_bottom, static_tf_front, static_tf_side,
        static_tf_imu,
        imu_node,
    ])