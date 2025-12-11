#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def tf_to_mat(tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    # quaternion xyzw -> matriz 4x4
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T

def mat_to_tf(T: np.ndarray, parent: str, child: str) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp.sec = 0  # será preenchido pelo broadcaster estático
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tx, ty, tz = T[:3, 3]
    tf.transform.translation.x = float(tx)
    tf.transform.translation.y = float(ty)
    tf.transform.translation.z = float(tz)
    # matriz -> quaternion (xyzw)
    m = T[:3, :3]
    qw = math.sqrt(max(0.0, 1.0 + m[0,0] + m[1,1] + m[2,2])) / 2.0
    qx = (m[2,1] - m[1,2]) / (4.0*qw) if qw != 0 else 0.0
    qy = (m[0,2] - m[2,0]) / (4.0*qw) if qw != 0 else 0.0
    qz = (m[1,0] - m[0,1]) / (4.0*qw) if qw != 0 else 0.0
    tf.transform.rotation.x = float(qx)
    tf.transform.rotation.y = float(qy)
    tf.transform.rotation.z = float(qz)
    tf.transform.rotation.w = float(qw)
    return tf
    
def average_quaternions(q_list):
    # q_list: lista de [x,y,z,w]
    M = np.zeros((4,4))
    for q in q_list:
        q = np.array(q, dtype=float)
        q = q / np.linalg.norm(q)
        M += np.outer(q, q)
    M /= len(q_list)
    eigvals, eigvecs = np.linalg.eigh(M)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    # volta em xyzw
    return [float(q_avg[0]), float(q_avg[1]), float(q_avg[2]), float(q_avg[3])]

def xyz_quat_to_mat(xyz, quat):
        tx, ty, tz = xyz
        x, y, z, w = quat
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
        ])
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [tx, ty, tz]
        return T

class BaseCamSolver(Node):
    def __init__(self):
        super().__init__('base_cam_solver')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('cam_frame',  'zed_left_camera_optical_frame')
        self.declare_parameter('cam_base_frame', 'zed_camera_link')
        self.declare_parameter('tag_frame',  'tag36h11:0')  # ou 'apriltag_0'
        self.declare_parameter('recompute',  False)  # se True, reavalia periodicamente
        self.declare_parameter('num_samples', 1)
        
        self.declare_parameter('base_to_tag_xyz', [0.01831, -0.11701, 0.02973])
        self.declare_parameter('base_to_tag_quat_xyzw', [0.70709, 0.00498, 0.00498, 0.70709])

        self.base = self.get_parameter('base_frame').get_parameter_value().string_value
        self.cam  = self.get_parameter('cam_frame').get_parameter_value().string_value
        self.cam_base  = self.get_parameter('cam_base_frame').get_parameter_value().string_value        
        self.tag  = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.recompute = self.get_parameter('recompute').get_parameter_value().bool_value
        self.num_samples = self.get_parameter('num_samples').value
        
        xyz = self.get_parameter('base_to_tag_xyz').get_parameter_value().double_array_value
        quat = self.get_parameter('base_to_tag_quat_xyzw').get_parameter_value().double_array_value
        self.T_base_tag = xyz_quat_to_mat(xyz, quat)

        self.buf = Buffer()
        self.tl  = TransformListener(self.buf, self)

        self.broadcaster = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(0.2, self.step)

        self.samples_T = []
        self.published = False

        self.get_logger().info(
            f"Waiting TF: {self.cam}<->{self.tag}."
            )

    def step(self):
        try:
            now = rclpy.time.Time()
            if not self.buf.can_transform(self.cam,  self.tag, rclpy.time.Time(), timeout=Duration(seconds=0.1)):
                return
            self.published = False
            self.get_logger().info(
                f"Calibrando {self.base} -> {self.cam} usando tag {self.tag}, "
                f"{self.num_samples} amostras."
            )

            t_cam_tag = self.buf.lookup_transform(
                self.cam, self.tag, now, timeout=Duration(seconds=0.1)
            )

            
            T_cam_tag  = tf_to_mat(t_cam_tag)
            T_tag_cam = np.linalg.inv(T_cam_tag)
            
            T_base_cam = self.T_base_tag @ T_tag_cam

            tf_aux = mat_to_tf(T_base_cam, self.base, self.cam)

            p = T_base_cam[:3, 3].tolist()
            q = [
                tf_aux.transform.rotation.x,
                tf_aux.transform.rotation.y,
                tf_aux.transform.rotation.z,
                tf_aux.transform.rotation.w,
            ]

            self.samples_T.append((p, q))

            self.get_logger().info(
            f"[{len(self.samples_T)}/{self.num_samples}] "
            f"sample p={np.round(p,3)}, q={np.round(q,3)}"
            )

            if len(self.samples_T) < self.num_samples:
                return
            
            ps = [s[0] for s in self.samples_T]
            qs = [s[1] for s in self.samples_T]
            p_avg = np.mean(np.array(ps), axis=0).tolist()
            q_avg = average_quaternions(qs)

            # monta transform final
            T_base_cam_final = xyz_quat_to_mat(p_avg, q_avg)

            now = rclpy.time.Time()
            t_cam_base_cam = self.buf.lookup_transform(
                self.cam_base,  # parent no lookup
                self.cam,       # child no lookup
                now,
                timeout=Duration(seconds=0.5)
            )
            T_cam_base_cam = tf_to_mat(t_cam_base_cam)
            T_cam_cam_base = np.linalg.inv(T_cam_base_cam)

            T_final = T_base_cam_final @ T_cam_cam_base

            tf_bc = mat_to_tf(T_final, self.base, self.cam_base)


            # publica em /tf_static (latch)
            self.broadcaster.sendTransform(tf_bc)
            self.get_logger().info(
                    f"=== CALIBRAÇÃO CONCLUÍDA ===\n"
                    f"base -> {self.cam}\n"
                    f"p = {p_avg}\n"
                    f"q_xyzw = {q_avg}\n"
                )
            
            if self.recompute:
                self.timer.timer_period_ns = int(5 * 1e9)
                self.samples_T.clear()
            if not self.recompute:
                self.timer.cancel()
            self.published = True

            
        except Exception as e:
            self.get_logger().warn(f"Waiting transforms: {e}")

def main():
    rclpy.init()
    node = BaseCamSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

