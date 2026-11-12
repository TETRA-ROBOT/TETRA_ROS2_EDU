#!/usr/bin/env python3
"""camera_info 보정값 보완 노드.

RealSense 컬러 센서의 보정 테이블이 비어 있는 카메라는 camera_info 의 K/D/P 가
NaN 으로 발행되어 AprilTag 위치(TF)가 NaN 이 되고 도킹이 동작하지 않는다.
이 노드는 camera_info 를 그대로 중계하되, 보정값이 유효하지 않을 때만
파라미터(params/camera_info_fix.yaml)의 값으로 채워서 다시 발행한다.
보정값이 정상인 카메라에서는 원본을 그대로 통과시킨다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoFix(Node):

    def __init__(self):
        super().__init__('camera_info_fix')
        self.ref_width = self.declare_parameter('width', 640).value
        self.ref_height = self.declare_parameter('height', 480).value
        self.fx = self.declare_parameter('fx', 616.0).value
        self.fy = self.declare_parameter('fy', 616.0).value
        self.cx = self.declare_parameter('cx', 320.0).value
        self.cy = self.declare_parameter('cy', 240.0).value
        self.d = list(self.declare_parameter('d', [0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self.warned = False

        self.pub = self.create_publisher(CameraInfo, 'camera_info_out', 10)
        self.sub = self.create_subscription(
            CameraInfo, 'camera_info_in', self.callback, qos_profile_sensor_data)

    @staticmethod
    def valid(msg):
        fx, fy = msg.k[0], msg.k[4]
        return all(math.isfinite(v) for v in msg.k) and fx > 0.0 and fy > 0.0

    def callback(self, msg):
        if not self.valid(msg):
            if not self.warned:
                self.warned = True
                self.get_logger().warn(
                    'camera_info 보정값이 유효하지 않아(NaN) 파라미터 값으로 대체합니다. '
                    '정확한 도킹을 위해 params/camera_info_fix.yaml 에 실제 캘리브레이션 값을 넣으세요.')
            sx = msg.width / float(self.ref_width) if msg.width else 1.0
            sy = msg.height / float(self.ref_height) if msg.height else 1.0
            fx, fy = self.fx * sx, self.fy * sy
            cx, cy = self.cx * sx, self.cy * sy
            msg.distortion_model = 'plumb_bob'
            msg.d = self.d
            msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraInfoFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
