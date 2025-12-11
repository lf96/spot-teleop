#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class NvbloxCloudToMoveIt(Node):
    """
    Node que:
      - lê um PointCloud2 do nvblox (ESDF / TSDF)
      - aplica filtro simples (range, z_min/z_max, decimação)
      - publica PointCloud2 em um frame fixo para o MoveIt consumir
    """

    def __init__(self):
        super().__init__("nvblox_cloud_to_moveit")

        # Parâmetros configuráveis
        self.declare_parameter(
            "input_cloud_topic", "/nvblox_node/pessimistic_static_esdf_pointcloud"
        )
        self.declare_parameter("output_cloud_topic", "/collision_cloud")
        self.declare_parameter("fixed_frame", "body")   # frame do planejador
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("z_min", -1.0)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("decimation", 1)  # pular pontos: 1 = usa todos

        self.input_topic = (
            self.get_parameter("input_cloud_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_cloud_topic").get_parameter_value().string_value
        )
        self.fixed_frame = (
            self.get_parameter("fixed_frame").get_parameter_value().string_value
        )
        self.max_range = (
            self.get_parameter("max_range").get_parameter_value().double_value
        )
        self.z_min = self.get_parameter("z_min").get_parameter_value().double_value
        self.z_max = self.get_parameter("z_max").get_parameter_value().double_value
        self.decimation = (
            self.get_parameter("decimation").get_parameter_value().integer_value
        )
        if self.decimation < 1:
            self.decimation = 1

        self.get_logger().info(
            f"Subscribing to: {self.input_topic}, publishing: {self.output_topic}, "
            f"frame: {self.fixed_frame}, max_range: {self.max_range} m, "
            f"z ∈ [{self.z_min}, {self.z_max}], decimation: {self.decimation}"
        )

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_cb,
            10,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        # pré-define campos XYZ do cloud de saída
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def cloud_cb(self, msg: PointCloud2):
        # Lê pontos XYZ do cloud do nvblox
        wanted_fields = ("x", "y", "z", "intensity")
        available = [f.name for f in msg.fields]
        fields = [f for f in wanted_fields if f in available]

        if not all(f in available for f in ("x", "y", "z")):
            self.get_logger().error(f"PointCloud não tem pelo menos x,y,z! Tem: {available}")
            return

        points_iter = point_cloud2.read_points(
            msg, field_names=fields, skip_nans=True
        )

        use_intensity = "intensity" in fields

        max_r2 = self.max_range * self.max_range
        filtered_points = []
        count_in = 0

        for p in points_iter:
            count_in += 1
            if use_intensity and len(p)==4:
                x, y, z, intensity = float(p[0]), float(p[1]), float(p[2]), float(p[3])
            else:
                x, y, z = float(p[0]), float(p[1]), float(p[2])

            # filtro de range
            r2 = x * x + y * y + z * z
            if r2 > max_r2:
                continue

            # filtro em Z
            if z < self.z_min or z > self.z_max:
                continue

            # decimação simples
            if self.decimation > 1 and (len(filtered_points) % self.decimation != 0):
                continue

            filtered_points.append((x, y, z))

        if not filtered_points:
            # nada pra publicar
            return

        # Header de saída: mesmo stamp, mas frame fixo (frame do planejador)
        header = msg.header
        header.frame_id = self.fixed_frame

        out_cloud = point_cloud2.create_cloud(header, self.fields, filtered_points)
        self.pub.publish(out_cloud)

        self.get_logger().debug(
            f"Recebido {count_in} pontos, publicado {len(filtered_points)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NvbloxCloudToMoveIt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()