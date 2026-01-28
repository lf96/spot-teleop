import subprocess
import time


class RosMonitor:
    def __init__(self, ui):
        self.ui = ui

    def wait_for_node(self, node):
        subprocess.call(["source", "/opt/ros/humble/setup.bash"], shell=True)
        while True:
            out = subprocess.check_output(["ros2", "node", "list"]).decode()
            if node in out:
                return
            time.sleep(1)

    def wait_for_topic(self, topic):
        subprocess.call(["source", "/opt/ros/humble/setup.bash"], shell=True)
        while True:
            out = subprocess.check_output(["ros2", "topic", "list"]).decode()
            if topic in out:
                return
            time.sleep(1)
