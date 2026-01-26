#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from tf2_ros import Buffer, TransformListener
import numpy as np

class NvbloxCloudToMoveIt(Node):
    """
    Node que:
      - lê um PointCloud2 do nvblox (ESDF / TSDF)
      - aplica filtro simples (range, z_min/z_max, decimação)
      - publica em planning scene
    """
   
    def tfstamped_to_mat(self, tfstamped):
        t = tfstamped.transform.translation
        q = tfstamped.transform.rotation
        x,y,z,w = q.x,q.y,q.z,q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
        ])
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [t.x,t.y,t.z]
        return T
    
    def __init__(self):
        super().__init__("nvblox_cloud_to_moveit")

        # ================= Parâmetros configuráveis ================= #
        # Topicos
        self.declare_parameter(
            "input_cloud_topic", "/nvblox_node/pessimistic_static_esdf_pointcloud"
        )
        self.declare_parameter("output_cloud_topic", "/collision_cloud")
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.input_topic = (
            self.get_parameter("input_cloud_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_cloud_topic").get_parameter_value().string_value
        )
        planning_scene_topic = (
            self.get_parameter("planning_scene_topic")
            .get_parameter_value().string_value
        )

        # Frames
        self.declare_parameter("fixed_frame", "body")   # frame do planejador
        self.declare_parameter("cam_frame", "zed_left_camera_optical_frame") # frame da camera
        self.fixed_frame = (
            self.get_parameter("fixed_frame").get_parameter_value().string_value
        )
        self.cam_frame = (
            self.get_parameter("cam_frame").get_parameter_value().string_value
        )

        # Filtros
        self.declare_parameter("max_range", 1.0)
        self.declare_parameter("z_min", -1.0)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("decimation", 1)  # pular pontos: 1 = usa todos
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

        # Refresh rate
        self.declare_parameter("publish_period_s", 10.0)
        self.publish_period = self.get_parameter(
            "publish_period_s"
        ).get_parameter_value().double_value

        # Collision
        self.declare_parameter("voxel_size", 0.03)
        self.declare_parameter("max_voxels", 200)
        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value
        self.max_voxels = self.get_parameter("max_voxels").get_parameter_value().integer_value
        if self.max_voxels < 1:
            self.max_voxels = 1

        # --- Voxel noise filter ---
        self.declare_parameter("min_points_per_voxel", 5)   # >= 2-10 normalmente funciona
        self.declare_parameter("voxel_mode", "2.5d")        # "2.5d" ou "3d"
        self.declare_parameter("min_pillar_height", 0.05)   # só no 2.5d: descarta pilares baixos (m)
        self.declare_parameter("max_pillar_height", 2.0)    # só no 2.5d: limita altura

        self.min_points_per_voxel = int(self.get_parameter("min_points_per_voxel").value)
        self.voxel_mode = str(self.get_parameter("voxel_mode").value).lower()
        self.min_pillar_height = float(self.get_parameter("min_pillar_height").value)
        self.max_pillar_height = float(self.get_parameter("max_pillar_height").value)

        # =========================================================== #

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

        self.ps_pub = self.create_publisher(
            PlanningScene,
            planning_scene_topic,
            10,
        )

        # pré-define campos XYZ do cloud de saída
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.last_publish_time = self.get_clock().now() - Duration(seconds=10.0)

    def cloud_cb(self, msg: PointCloud2):
        # Lê pontos XYZ do cloud do nvblox
        now = self.get_clock().now()
        if (now - self.last_publish_time) < Duration(seconds=self.publish_period):
            return
        if not self.buf.can_transform(self.fixed_frame,  msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1)):
            self.get_logger().info(
            f"Sem transformacao {self.fixed_frame} -> {msg.header.frame_id}"
            )
            
            return
        transform = self.buf.lookup_transform(
                self.fixed_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        T_fixed_from_cloud = self.tfstamped_to_mat(transform)
        self.get_logger().info(
            f"Transformacao {self.fixed_frame} -> {msg.header.frame_id}\n"
            f"T=\n{T_fixed_from_cloud}"
            )

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

        filtered_points = []
        xs = []
        ys = []
        zs = []
        count_in = 0
        filtered_items = [0 , 0 , 0]

        for p in points_iter:
            count_in += 1
            if use_intensity and len(p)==4:
                x, y, z, intensity = float(p[0]), float(p[1]), float(p[2]), float(p[3])
            else:
                x, y, z = float(p[0]), float(p[1]), float(p[2])

            p_fixed = T_fixed_from_cloud @ [x,y,z,1]
            xF,yF,zF = p_fixed[:3]
            dist = np.linalg.norm(p_fixed[:3])

            # filtro de range
            if dist > self.max_range:
                filtered_items[0] += 1
                continue

            # filtro em Z
            if zF < self.z_min or zF > self.z_max:
                filtered_items[1] += 1
                continue

            # decimação simples
            if self.decimation > 1 and (len(filtered_points) % self.decimation != 0):
                filtered_items[2] += 1
                continue

            filtered_points.append((xF,yF,zF))
            xs.append(xF)
            ys.append(yF)
            zs.append(zF)

        if not filtered_points:
            # nada pra publicar
            return

        # Header de saída: mesmo stamp, mas frame fixo (frame do planejador)
        header = msg.header
        header.frame_id = self.fixed_frame

        out_cloud = point_cloud2.create_cloud(header, self.fields, filtered_points)
        self.pub.publish(out_cloud)

        self.get_logger().info(
            f"Recebidos {count_in} pontos, publicados {len(filtered_points)}\n"
            f"Filtros aplicados:\nrange={filtered_items[0]}\nz={filtered_items[1]}\ndecimation={filtered_items[2]}"
        )

        vs = self.voxel_size
        min_n = max(1, self.min_points_per_voxel)

        # -------------------------
        # A) modo 2.5D (ix,iy) com min/max z e contagem
        # -------------------------
        if self.voxel_mode == "2.5d":
            # (ix,iy) -> dict com bounds e count
            voxels = {}  # key -> {min_x,max_x,min_y,max_y,min_z,max_z,count}
            for x, y, z in zip(xs, ys, zs):
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                ix = int(math.floor(x / vs))
                iy = int(math.floor(y / vs))
                key = (ix, iy)

                voxel_min_x = ix * vs
                voxel_max_x = (ix + 1) * vs
                voxel_min_y = iy * vs
                voxel_max_y = (iy + 1) * vs

                if key not in voxels:
                    voxels[key] = [voxel_min_x, voxel_max_x,
                                voxel_min_y, voxel_max_y,
                                z, z, 1]  # min_z,max_z,count
                else:
                    v = voxels[key]
                    v[4] = min(v[4], z)
                    v[5] = max(v[5], z)
                    v[6] += 1

            # filtra por densidade (ruído)
            voxel_items = []
            dropped_sparse = 0
            dropped_height = 0

            for key, v in voxels.items():
                count = v[6]
                if count < min_n:
                    dropped_sparse += 1
                    continue

                height = v[5] - v[4]
                if height < self.min_pillar_height:
                    dropped_height += 1
                    continue

                # limita altura do pilar pra não virar “parede infinita”
                if height > self.max_pillar_height:
                    mid = 0.5 * (v[4] + v[5])
                    v[4] = mid - 0.5 * self.max_pillar_height
                    v[5] = mid + 0.5 * self.max_pillar_height

                voxel_items.append((key, v))

            self.get_logger().info(
                f"Voxels 2.5D: total={len(voxels)} kept={len(voxel_items)} "
                f"dropped_sparse(<{min_n})={dropped_sparse} dropped_height={dropped_height}"
            )
            def score_25d(item):
                (_key, v) = item
                min_x, max_x, min_y, max_y, min_z, max_z, count = v
                cx = 0.5 * (min_x + max_x)
                cy = 0.5 * (min_y + max_y)

                d2 = cx*cx + cy*cy

                return d2
            
            voxel_items.sort(key=score_25d)

        # -------------------------
        # B) modo 3D (ix,iy,iz) com contagem
        # -------------------------
        elif self.voxel_mode == "3d":
            # (ix,iy,iz) -> count
            counts = {}
            for x, y, z in zip(xs, ys, zs):
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                ix = int(math.floor(x / vs))
                iy = int(math.floor(y / vs))
                iz = int(math.floor(z / vs))
                key = (ix, iy, iz)
                counts[key] = counts.get(key, 0) + 1

            voxel_items = []
            dropped_sparse = 0
            for key, c in counts.items():
                if c < min_n:
                    dropped_sparse += 1
                    continue
                voxel_items.append((key, c))

            self.get_logger().info(
                f"Voxels 3D: total={len(counts)} kept={len(voxel_items)} dropped_sparse(<{min_n})={dropped_sparse}"
            )
            def score_3d(item):
                (key, count) = item
                ix, iy, iz = key
                cx = (ix + 0.5) * vs
                cy = (iy + 0.5) * vs
                d2 = cx*cx + cy*cy
                return d2

            voxel_items.sort(key=score_3d)

        else:
            self.get_logger().warn(f"voxel_mode inválido: {self.voxel_mode}. Use '2.5d' ou '3d'.")
            return

        # limita quantidade
        if len(voxel_items) > self.max_voxels:
            self.get_logger().warn(f"Priorizando voxels proximos: {self.max_voxels}/{len(voxel_items)}")
            voxel_items = voxel_items[: self.max_voxels]


        co = CollisionObject()
        co.header.frame_id = self.fixed_frame  # frame_id = body (mesmo do nvblox)
        co.header.stamp = msg.header.stamp
        co.id = "nvblox_env"
        co.operation = CollisionObject.ADD
        if self.voxel_mode == "2.5d":
            for (ix, iy), v in voxel_items:
                min_x, max_x, min_y, max_y, min_z, max_z, count = v
                size_x = max_x - min_x
                size_y = max_y - min_y
                size_z = max(max_z - min_z, vs * 0.5)

                center_x = (max_x + min_x) / 2.0
                center_y = (max_y + min_y) / 2.0
                center_z = (max_z + min_z) / 2.0

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [float(size_x), float(size_y), float(size_z)]

                pose = Pose()
                pose.position.x = float(center_x)
                pose.position.y = float(center_y)
                pose.position.z = float(center_z)
                pose.orientation.w = 1.0

                co.primitives.append(box)
                co.primitive_poses.append(pose)

            self.get_logger().info(
                f"CollisionObject 'nvblox_env' atualizado com {len(co.primitives)} caixas "
                f"(voxel_size={vs:.2f} m)"
            )

        if self.voxel_mode == "3d":
            for (ix, iy, iz), count in voxel_items:
                min_x = ix * vs
                max_x = (ix + 1) * vs
                min_y = iy * vs
                max_y = (iy + 1) * vs
                min_z = iz * vs
                max_z = (iz + 1) * vs

                center_x = (min_x + max_x) / 2.0
                center_y = (min_y + max_y) / 2.0
                center_z = (min_z + max_z) / 2.0

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [vs, vs, vs]

                pose = Pose()
                pose.position.x = float(center_x)
                pose.position.y = float(center_y)
                pose.position.z = float(center_z)
                pose.orientation.w = 1.0

                co.primitives.append(box)
                co.primitive_poses.append(pose)

        # ---------- PlanningScene diff ----------
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)

        self.ps_pub.publish(ps)
        self.last_publish_time = now

    



def main(args=None):
    rclpy.init(args=args)
    node = NvbloxCloudToMoveIt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()