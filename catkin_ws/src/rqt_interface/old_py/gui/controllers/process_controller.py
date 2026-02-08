import subprocess
import threading
import sys
import os

class ProcessController:
    def __init__(self, ui):
        self.ui = ui
        self.process = None

    def start(self, script_path):
        if self.process:
            print(f"{script_path} Process already running")
            return

        self.process = subprocess.Popen(
            ["bash", "-lc", script_path],
            stdin=sys.stdin,
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
            preexec_fn= os.setsid
        )

    def stop(self):
        if self.process:
            print("Stopping process")
            self.process.terminate()
            self.process = None
