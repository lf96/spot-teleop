import yaml
from process_controller import ProcessController
from ros_monitor import RosMonitor
import subprocess
import time
import sys
import os

class PipelineController:
    def __init__(self):
        self.processes = {}

        with open("/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/config/pipelines.yaml") as f:
            self.cfg = yaml.safe_load(f)["pipelines"]

    def start_pipeline(self, name):
        print(f"[PIPELINE] Starting pipeline: {name}")
        pipeline = self.cfg[name]

        for stage in pipeline:
            stage_name = stage["name"]
            script = stage["script"]

            # TODO: Fix waiting mechanism
            if "wait_for" in stage:
                self._wait(stage["wait_for"])
            
            print(f"[PIPELINE] Starting stage: {stage['name']} with script: {script}")
            proc = ProcessController()
            proc.start(script)
            
            self.processes[stage_name] = proc

    def _wait(self, cond):
        print(f"[PIPELINE] Waiting for condition: {cond}")

        if "process" in cond:
            proc_name = cond["process"]
            if proc_name not in self.processes:
                raise RuntimeError(f"Process '{proc_name}' not started yet")

            self._wait_for_process_ready(proc_name)

        # TODO: Implement ROS condition waits
        # if "ros_node" in cond:
        #     self.ros.wait_for_node(cond["ros_node"])

        # if "ros_topic" in cond:
        #     self.ros.wait_for_topic(cond["ros_topic"])

    def _wait_for_process_ready(self, proc_name):
        """
        Core-specific readiness check
        """
        if proc_name == "core":
            print("[PIPELINE] Waiting for Isaac core readiness signal")
            self._wait_for_core_ready()


    def _wait_for_core_ready(self):
        """
        Isaac Sim readiness handshake
        """
        time.sleep(10)
        READY_FILE = "/tmp/isaac_core_ready"

        while True:
            result = subprocess.run(
                ["docker-compose", "exec", "isaac-sim", "test", "-f", READY_FILE],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if result.returncode == 0:
                print("[PIPELINE] Isaac core READY")
                return
            time.sleep(1)

    def stop(self):
        for name, proc in self.processes.items():
            print(f"[PIPELINE] Stopping process: {name}")
            proc.stop()
        self.process = subprocess.Popen(
            ["bash", "-lc", "/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/shutdown/00_all.sh"],
            stdin=sys.stdin,
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
            preexec_fn= os.setsid
        )
        self.processes = {}