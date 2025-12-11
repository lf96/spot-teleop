#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DepthMaskFilter(Node):
    def __init__(self):
        super().__init__('depth_mask_filter')

        # ----- Parâmetros configuráveis -----
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('mask_topic',  '/cumotion/camera_1/robot_mask')
        self.declare_parameter('output_topic', '/zed/zed_node/depth/masked_depth')
        # se True: zera onde mask == 0 (inversão); se False: zera onde mask != 0
        self.declare_parameter('invert_mask', False)

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        mask_topic  = self.get_parameter('mask_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.invert_mask = self.get_parameter('invert_mask').get_parameter_value().bool_value

        self.get_logger().info(f"Subscribing depth: {depth_topic}")
        self.get_logger().info(f"Subscribing mask:  {mask_topic}")
        self.get_logger().info(f"Publishing filtered depth: {output_topic}")
        self.get_logger().info(f"Invert mask: {self.invert_mask}")

        # ----- Publisher -----
        self.pub_depth = self.create_publisher(Image, output_topic, 10)

        # ----- Subscribers + sincronizador -----
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=10)
        self.mask_sub  = Subscriber(self, Image, mask_topic,  qos_profile=10)

        # sincronização aproximada
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.mask_sub],
            queue_size=10,
            slop=0.05  # 50 ms de tolerância
        )
        self.sync.registerCallback(self.callback)

        self.warned_encodings = False

    def callback(self, depth_msg: Image, mask_msg: Image):
        # Checar encodings
        if not self.warned_encodings:
            self.get_logger().info(f"depth encoding: {depth_msg.encoding}")
            self.get_logger().info(f"mask  encoding: {mask_msg.encoding}")
            self.warned_encodings = True

        if depth_msg.encoding not in ['32FC1', 'TYPE_32FC1']:
            self.get_logger().warn(f"Depth encoding inesperado: {depth_msg.encoding} (esperado 32FC1)")
        if mask_msg.encoding not in ['mono8', '8UC1']:
            self.get_logger().warn(f"Mask encoding inesperado: {mask_msg.encoding} (esperado mono8/8UC1)")

        # Converter para numpy
        try:
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                (depth_msg.height, depth_msg.width)
            )
        except Exception as e:
            self.get_logger().error(f"Erro ao converter depth para numpy: {e}")
            return

        try:
            mask = np.frombuffer(mask_msg.data, dtype=np.uint8).reshape(
                (mask_msg.height, mask_msg.width)
            )
        except Exception as e:
            self.get_logger().error(f"Erro ao converter mask para numpy: {e}")
            return

        # Conferir se tamanhos batem
        if depth.shape != mask.shape:
            self.get_logger().warn(
                f"Dimensão depth {depth.shape} != mask {mask.shape}. "
                f"Ignorando este par."
            )
            return

        # ----- Aplicar máscara -----
        depth_filtered = depth.copy()

        if self.invert_mask:
            # Zera onde mask == 0
            invalid = (mask == 0)
        else:
            # Zera onde mask != 0  (por ex: 1 = robô)
            invalid = (mask != 0)

        # marcar como inválido: NaN é uma boa escolha pro NVBlox
        depth_filtered[invalid] = np.nan

        # ----- Montar nova mensagem -----
        out_msg = Image()
        out_msg.header = depth_msg.header
        out_msg.height = depth_msg.height
        out_msg.width = depth_msg.width
        out_msg.encoding = depth_msg.encoding  # mantém 32FC1
        out_msg.is_bigendian = depth_msg.is_bigendian
        out_msg.step = depth_msg.step
        out_msg.data = depth_filtered.tobytes()

        self.pub_depth.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMaskFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
