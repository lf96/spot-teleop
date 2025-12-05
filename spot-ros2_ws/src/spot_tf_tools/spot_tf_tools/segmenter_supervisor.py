#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import subprocess
import shlex
import time
import signal
import os


class SegmenterSupervisor(Node):
    def __init__(self):
        super().__init__("segmenter_supervisor")

        # comando para subir o robot_segmenter_node
        # Exemplo de default (ajuste para o seu caso real):
        default_cmd = (
            "/home/spot-teleop/spot-ros2_ws/install/isaac_ros_cumotion/lib/isaac_ros_cumotion/robot_segmenter_node "            "--ros-args "
            "-r __node:=cumotion_robot_segmenter "
            "--params-file /home/spot-teleop/spot-ros2_ws/src/spot_moveit_config/config/robot_segmenter_params.yaml"
        )
        self.declare_parameter("segmenter_cmd", default_cmd)

        self.segmenter_cmd = self.get_parameter("segmenter_cmd").value
        self.proc: subprocess.Popen | None = None

        self.get_logger().info(f"Segmenter command: {self.segmenter_cmd}")

        # sobe o segmenter na inicialização
        self.kill_stray_segmenters()
        self.start_segmenter()

        # subscreve na flag de calibração
        self.sub = self.create_subscription(
            Bool,
            "/cumotion/calibration_changed",
            self.on_calibration_changed,
            10,
        )

    def kill_stray_segmenters(self):
        """
        Mata qualquer processo antigo de robot_segmenter_node que não
        esteja sendo gerenciado por este supervisor.
        """
        try:
            out = subprocess.check_output(
                ["pgrep", "-f", "isaac_ros_cumotion.*robot_segmenter_node"],
                text=True
            )
        except subprocess.CalledProcessError:
            # pgrep retorna exit code !=0 se não achar nada
            self.get_logger().info("No stray robot_segmenter_node processes found.")
            return

        pids = [int(x) for x in out.split() if x.strip().isdigit()]
        for pid in pids:
            # se já temos um proc e for o mesmo PID, não mata
            if self.proc is not None and pid == self.proc.pid:
                continue
            self.get_logger().warn(f"Killing stray robot_segmenter_node PID {pid}")
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def start_segmenter(self):
        if self.proc is not None and self.proc.poll() is None:
            self.get_logger().warn("Segmenter already running, not starting another.")
            return
        self.get_logger().info("Starting robot_segmenter process...")
        try:
            self.proc = subprocess.Popen(
                shlex.split(self.segmenter_cmd),
                stdout=None,
                stderr=None,
            )
            self.get_logger().info(f"robot_segmenter started with PID {self.proc.pid}")
        except Exception as e:
            self.get_logger().error(f"Failed to start segmenter: {e}")
            self.proc = None

    def stop_segmenter(self):
        if self.proc is None:
            self.get_logger().warn("No segmenter process to stop.")
            return
        if self.proc.poll() is not None:
            self.get_logger().info("Segmenter process already exited.")
            self.proc = None
            return

        self.get_logger().info("Stopping robot_segmenter process...")
        try:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Segmenter did not exit, killing...")
                self.proc.kill()
            self.get_logger().info(f"robot_segmenter with PID {self.proc.pid} stopped.")
        except Exception as e:
            self.get_logger().error(f"Error while stopping segmenter: {e}")
        finally:
            self.proc = None

    def on_calibration_changed(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().warn("Calibration changed flag received. Restarting segmenter...")
        self.stop_segmenter()
        # pequeno delay pra garantir que portas/tópicos foram liberados
        time.sleep(3.0)
        self.kill_stray_segmenters()
        self.start_segmenter()

    def destroy_node(self):
        # garante que o filho morra junto
        self.stop_segmenter()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmenterSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
