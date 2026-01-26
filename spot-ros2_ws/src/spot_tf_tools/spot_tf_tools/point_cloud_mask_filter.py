#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    # rotation matrix 3x3 from quaternion xyzw
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


class PointCloudMaskFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_mask_filter")

        # ---------- Params ----------
        self.declare_parameter("cloud_topic", "/zed/zed_node/point_cloud/cloud_registered")
        self.declare_parameter("mask_topic",  "/cumotion/camera_1/robot_mask")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/depth/camera_info")
        self.declare_parameter("output_cloud_topic", "/zed/zed_node/point_cloud/masked_cloud")

        # Frame da câmera para projeção (normalmente optical frame)
        self.declare_parameter("camera_frame", "zed_left_camera_optical_frame")

        # Se True: remove onde mask == 0. Se False: remove onde mask != 0
        self.declare_parameter("invert_mask", False)

        # Se quiser limitar carga
        self.declare_parameter("max_points", 0)   # 0 = sem limite
        self.declare_parameter("decimation", 1)        # 1 = usa todos

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_cloud_topic = self.get_parameter("output_cloud_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.invert_mask = bool(self.get_parameter("invert_mask").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.decimation = max(1, int(self.get_parameter("decimation").value))

        self.get_logger().info(f"Cloud in : {self.cloud_topic}")
        self.get_logger().info(f"Mask in  : {self.mask_topic}")
        self.get_logger().info(f"CamInfo  : {self.camera_info_topic}")
        self.get_logger().info(f"Cloud out: {self.output_cloud_topic}")
        self.get_logger().info(f"camera_frame: {self.camera_frame}")
        self.get_logger().info(f"invert_mask: {self.invert_mask}")

        # ---------- TF ----------
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # ---------- Pub/Sub ----------
        self.pub = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)

        self.cloud_sub = Subscriber(self, PointCloud2, self.cloud_topic, qos_profile=10)
        self.mask_sub  = Subscriber(self, Image,      self.mask_topic,  qos_profile=10)
        self.info_sub  = Subscriber(self, CameraInfo, self.camera_info_topic, qos_profile=10)

        self.sync = ApproximateTimeSynchronizer(
            [self.cloud_sub, self.mask_sub, self.info_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.cb)

        self.warned = False

    def cb(self, cloud: PointCloud2, mask_msg: Image, cam_info: CameraInfo):
        # --- mask decode ---
        if mask_msg.encoding not in ["mono8", "8UC1"]:
            if not self.warned:
                self.get_logger().warn(f"mask encoding inesperado: {mask_msg.encoding} (esperado mono8/8UC1)")
        try:
            mask = np.frombuffer(mask_msg.data, dtype=np.uint8).reshape((mask_msg.height, mask_msg.width))
        except Exception as e:
            self.get_logger().error(f"Erro lendo mask: {e}")
            return

        # regra de invalidez
        if self.invert_mask:
            invalid_pixel = (mask == 0)
        else:
            invalid_pixel = (mask != 0)

        # --- intrinsics ---
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn("CameraInfo sem intrínsecos válidos (fx/fy = 0).")
            return

        # Se cloud for organizado e bater resolução, dá pra indexar direto
        organized = (cloud.height > 1 and cloud.width > 1 and
                     cloud.height == mask_msg.height and cloud.width == mask_msg.width)

        # Campos disponíveis
        available = [f.name for f in cloud.fields]
        if not all(n in available for n in ("x", "y", "z")):
            self.get_logger().error(f"Cloud não tem x,y,z. Tem: {available}")
            return

        # Vamos manter pelo menos xyz (pode estender depois)
        field_names = ("x", "y", "z")
        pts = point_cloud2.read_points(cloud, field_names=field_names, skip_nans=True)

        # Limites
        out_points = []
        count = 0

        # ---------- Modo 1: organizado ----------
        if organized:
            # Atenção: read_points vem em ordem row-major (v,u)
            for idx, (x, y, z) in enumerate(pts):
                if self.decimation > 1 and (idx % self.decimation) != 0:
                    continue

                # pixel correspondente
                v = idx // cloud.width
                u = idx - v * cloud.width
                if v >= mask_msg.height or u >= mask_msg.width:
                    continue

                if invalid_pixel[v, u]:
                    continue

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                out_points.append((float(x), float(y), float(z)))
                count += 1
                if self.max_points > 0 and count >= self.max_points:
                    break

        # ---------- Modo 2: desorganizado (projeção) ----------
        else:
            # Precisa levar pontos pro frame óptico da câmera para projetar (X right, Y down, Z forward)
            src_frame = cloud.header.frame_id
            dst_frame = self.camera_frame

            try:
                if not self.tf_buf.can_transform(dst_frame, src_frame, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.2)):
                    self.get_logger().warn(f"Sem TF {dst_frame} <- {src_frame}")
                    return

                tf = self.tf_buf.lookup_transform(dst_frame, src_frame, rclpy.time.Time(),
                                                  timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                q = tf.transform.rotation
                R = quat_to_rot(q.x, q.y, q.z, q.w)
                T = np.array([t.x, t.y, t.z], dtype=np.float64)
            except Exception as e:
                self.get_logger().warn(f"Erro TF: {e}")
                return

            for idx, (x, y, z) in enumerate(pts):
                if self.decimation > 1 and (idx % self.decimation) != 0:
                    continue

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                p_src = np.array([x, y, z], dtype=np.float64)
                p_cam = R @ p_src + T  # no optical frame

                X, Y, Z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
                if not math.isfinite(Z) or Z <= 0.0:
                    continue

                u = int(fx * (X / Z) + cx)
                v = int(fy * (Y / Z) + cy)

                if u < 0 or v < 0 or u >= mask_msg.width or v >= mask_msg.height:
                    continue

                if invalid_pixel[v, u]:
                    continue

                # Mantém ponto no frame original (mais seguro para downstream)
                out_points.append((float(x), float(y), float(z)))
                count += 1
                if self.max_points > 0 and count >= self.max_points:
                    break

        # Publica cloud filtrado
        if not out_points:
            self.get_logger().warn("Nenhum ponto sobreviveu ao filtro.")
            return

        header = cloud.header  # mantém frame original
        out_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out_cloud = point_cloud2.create_cloud(header, out_fields, out_points)
        self.pub.publish(out_cloud)

        if not self.warned:
            self.get_logger().info(
                f"Modo={'organized' if organized else 'projected'} | "
                f"mask={mask_msg.width}x{mask_msg.height} | "
                f"out_points={len(out_points)} | cloud_frame={cloud.header.frame_id}"
            )
            self.warned = True


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMaskFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
