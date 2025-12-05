#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool


def quat_to_np_xyzw(q):
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def quat_angle_deg(q1_xyzw, q2_xyzw):
    # garante normalização
    def norm(q):
        n = np.linalg.norm(q)
        return q / n if n > 0 else q

    q1 = norm(q1_xyzw)
    q2 = norm(q2_xyzw)
    dot = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
    # menor ângulo entre as duas orientações
    angle_rad = 2.0 * math.acos(abs(dot))
    return math.degrees(angle_rad)


class TfCalibMonitor(Node):
    def __init__(self):
        super().__init__("tf_calib_monitor")

        self.declare_parameter("base_frame", "base")
        self.declare_parameter("cam_frame", "zed_left_camera_optical_frame")
        self.declare_parameter("check_period", 0.2)            # 5Hz
        # Parâmetros da janela / histerese
        self.declare_parameter('window_size', 5)
        self.declare_parameter('max_trans_std_m', 0.01)        # 1 cm de jitter aceitável
        self.declare_parameter('max_rot_std_deg', 2.0)         # 2 graus de jitter
        self.declare_parameter('min_delta_trans_m', 0.05)      # só atualiza se mover > 5 cm
        self.declare_parameter('min_delta_rot_deg', 3.0)       # ou rotacionar > 3 graus

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.cam_frame = self.get_parameter("cam_frame").get_parameter_value().string_value
        self.check_period = self.get_parameter('check_period').value
        self.window_size      = self.get_parameter('window_size').value
        self.max_trans_std    = self.get_parameter('max_trans_std_m').value
        self.max_rot_std_deg  = self.get_parameter('max_rot_std_deg').value
        self.min_delta_trans  = self.get_parameter('min_delta_trans_m').value
        self.min_delta_rot    = self.get_parameter('min_delta_rot_deg').value

        self.get_logger().info(
            f"Monitoring TF {self.base_frame} -> {self.cam_frame} "
            f"(pos>{self.min_delta_trans:.3f} m, ang>{self.min_delta_rot:.1f} deg)"
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_tf_translation = []
        self.last_tf_rotation = []
        self.last_trans = None   # np.array(3)
        self.last_quat  = None   # np.array(4, xyzw)

        # publisher da flag
        self.changed_pub = self.create_publisher(Bool, "/cumotion/calibration_changed", 10)

        # timer periódico
        self.timer = self.create_timer(self.check_period, self.check_tf)

    def check_tf(self):
        try:
            now = rclpy.time.Time()
            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.cam_frame,
                now,
                timeout=Duration(seconds=0.2),
            ):
                self.get_logger().debug("No TF available yet.")
                return

            tf_bc: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.cam_frame,
                now,
                timeout=Duration(seconds=0.2),
            )

            t = tf_bc.transform.translation
            q = tf_bc.transform.rotation
            p_vec = np.array([t.x, t.y, t.z], dtype=float)
            q_vec = quat_to_np_xyzw(q)

            if len(self.last_tf_translation) < self.window_size:
                self.get_logger().debug(f"Appending initial window data {len(self.last_tf_translation)}/{self.window_size}")
                self.last_tf_translation.append(p_vec)
                self.last_tf_rotation.append(q_vec)
                return
            
            self.last_tf_translation.pop(0)
            self.last_tf_rotation.pop(0)
            self.last_tf_translation.append(p_vec)
            self.last_tf_rotation.append(q_vec)

            trans_arr = np.stack(self.last_tf_translation, axis=0)
            trans_mean = trans_arr.mean(axis=0)
            trans_dev = np.linalg.norm(trans_arr - trans_mean, axis=1)
            trans_std = float(trans_dev.std())

            q_ref = self.last_tf_rotation[0] / np.linalg.norm(self.last_tf_rotation[0])
            angles = []
            for qi in self.last_tf_rotation:
                angles.append(quat_angle_deg(q_ref, qi))
            rot_std = float(np.std(angles))

            if trans_std > self.max_trans_std or rot_std > self.max_rot_std_deg:
                # TF ainda “dançando” por ruído → não atualiza
                self.get_logger().debug(f"Waiting stabilization...")
                return
            
            q_arr = np.stack(self.last_tf_rotation, axis=0)  # (N,4)
            q_mean = q_arr.mean(axis=0)
            q_mean = q_mean / np.linalg.norm(q_mean)

            if self.last_trans is None:
                self.get_logger().debug(f"Initial position / orientation saved!")
                self.last_trans = trans_mean
                self.last_quat = q_mean
                return
            
            delta_trans = np.linalg.norm(trans_mean - self.last_trans)
            delta_rot   = quat_angle_deg(self.last_quat, q_mean)

            if delta_trans < self.min_delta_trans and delta_rot < self.min_delta_rot:
                # mudança pequena, ignora
                self.get_logger().debug(
                "Small change detected and ignored:\n"
                f"delta_pos={delta_trans:.4f} m, delta_ang={delta_rot:.2f} deg"
                )
                return

            self.get_logger().warn(
                f"Calibration change detected! "
                f"Δpos={delta_trans:.3f} m, Δang={delta_rot:.1f} deg"
            )
            # publica flag
            msg = Bool()
            msg.data = True
            self.changed_pub.publish(msg)
            self.last_trans = trans_mean
            self.last_quat = q_mean


        except Exception as e:
            self.get_logger().warn(f"Error while checking TF: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TfCalibMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
