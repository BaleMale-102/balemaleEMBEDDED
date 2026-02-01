#!/usr/bin/env python3
"""
simple_marker_drive.py

간단한 마커 기반 주행 테스트
- 회전(wz) 대신 횡이동(vy)으로 보정
- rel_z < threshold 면 도달 정지
"""

import rclpy
from rclpy.node import Node
from rc_interfaces.msg import MarkerStatus, DriveCmd

class SimpleMarkerDrive(Node):
    def __init__(self):
        super().__init__('simple_marker_drive')
        
        # 파라미터
        self.declare_parameter('target_marker_id', 1)
        self.declare_parameter('vx', 0.003)           # 전진 속도 (매우 느림)
        self.declare_parameter('vy_gain', 0.01)       # 횡이동 게인
        self.declare_parameter('max_vy', 0.003)       # 최대 횡이동
        self.declare_parameter('reach_distance', 0.30) # 추가 전진 시작 거리 (m)
        self.declare_parameter('extra_forward_sec', 1.5) # 마커 사라진 후 추가 전진 시간
        
        self.target_id = self.get_parameter('target_marker_id').value
        self.vx = self.get_parameter('vx').value
        self.vy_gain = self.get_parameter('vy_gain').value
        self.max_vy = self.get_parameter('max_vy').value
        self.reach_dist = self.get_parameter('reach_distance').value
        self.extra_sec = self.get_parameter('extra_forward_sec').value
        
        self.enabled = False
        self.reached = False
        
        # 추가 전진 상태
        self.last_z = 999.0           # 마지막 마커 거리
        self.extra_start_time = None  # 추가 전진 시작 시간
        
        # Publishers
        self.pub_cmd = self.create_publisher(DriveCmd, '/control/drive_cmd', 10)
        
        # Subscribers
        self.sub_marker = self.create_subscription(
            MarkerStatus, '/perception/marker_status', self.cb_marker, 10)
        
        # Timer (30Hz)
        self.timer = self.create_timer(0.033, self.control_loop)
        
        # 최신 마커 데이터
        self.marker = None
        self.marker_time = None
        
        self.get_logger().info(f'SimpleMarkerDrive: target={self.target_id}, vx={self.vx}, reach={self.reach_dist}m')
        self.get_logger().info('Publish to /simple_drive/enable (Bool) to start')
        
        # Enable subscriber
        from std_msgs.msg import Bool
        self.sub_enable = self.create_subscription(
            Bool, '/simple_drive/enable', self.cb_enable, 10)
    
    def cb_enable(self, msg):
        self.enabled = msg.data
        if msg.data:
            self.reached = False
            self.last_z = 999.0
            self.extra_start_time = None
            self.get_logger().info('Enabled - starting drive')
        else:
            self.get_logger().info('Disabled - stopping')
    
    def cb_marker(self, msg: MarkerStatus):
        if msg.valid and msg.id == self.target_id:
            self.marker = msg
            self.marker_time = self.get_clock().now()
    
    def control_loop(self):
        cmd = DriveCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        if not self.enabled or self.reached:
            cmd.enable = False
            cmd.vx = cmd.vy = cmd.wz = 0.0
            self.pub_cmd.publish(cmd)
            return
        
        # 마커 유효성 체크 (0.5초 이내)
        now = self.get_clock().now()
        marker_valid = (self.marker is not None and 
                        self.marker_time is not None and
                        (now - self.marker_time).nanoseconds < 500_000_000)
        
        if not marker_valid:
            # 마커가 가까웠다가 사라짐 → 추가 전진
            if self.last_z < self.reach_dist:
                import time
                now_sec = time.time()
                
                if self.extra_start_time is None:
                    self.extra_start_time = now_sec
                    self.get_logger().info(f'Marker lost at z={self.last_z:.2f}m, extra forward {self.extra_sec}s')
                
                elapsed = now_sec - self.extra_start_time
                
                if elapsed < self.extra_sec:
                    # 추가 전진 중
                    cmd.enable = True
                    cmd.vx = self.vx
                    cmd.vy = 0.0
                    cmd.wz = 0.0
                    cmd.source = 'extra_forward'
                    self.get_logger().info(f'Extra forward: {elapsed:.1f}/{self.extra_sec}s', throttle_duration_sec=0.3)
                    self.pub_cmd.publish(cmd)
                    return
                else:
                    # 추가 전진 완료 → 정지
                    self.reached = True
                    self.enabled = False
                    cmd.enable = False
                    cmd.vx = cmd.vy = cmd.wz = 0.0
                    self.get_logger().info(f'REACHED after extra forward!')
                    self.pub_cmd.publish(cmd)
                    return
            
            # 마커 멀리서 사라짐 → 느리게 직진
            cmd.enable = True
            cmd.vx = self.vx * 0.5
            cmd.vy = 0.0
            cmd.wz = 0.0
            cmd.source = 'blind'
            self.get_logger().info('No marker - slow forward', throttle_duration_sec=1.0)
            self.pub_cmd.publish(cmd)
            return
        
        rel_x = self.marker.rel_x    # 좌우 (양수 = 오른쪽)
        rel_z = self.marker.rel_z    # 거리
        self.last_z = rel_z          # 기억
        
        # (도달 체크는 마커 사라질 때 함 - 카메라 한계)
        
        # 횡이동 보정 (rel_x 반대 방향으로)
        vy = -self.vy_gain * rel_x
        vy = max(-self.max_vy, min(self.max_vy, vy))
        
        cmd.enable = True
        cmd.vx = self.vx
        cmd.vy = float(vy)
        cmd.wz = 0.0  # 회전 안 함
        cmd.source = f'marker_{self.target_id}'
        
        self.get_logger().info(f'z={rel_z:.2f} x={rel_x:.3f} -> vx={cmd.vx:.3f} vy={cmd.vy:.3f}', 
                               throttle_duration_sec=0.5)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = SimpleMarkerDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
