from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout,
    QPushButton, QTextEdit, QHBoxLayout
)
from controllers.pipeline_controller import PipelineController
from PySide6.QtCore import QProcess, Qt
import os

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spot Teleop Control")
        self.resize(900, 600)

        self.pipeline = PipelineController(self)

        start_btn = QPushButton("Start Pipeline")
        start_btn.clicked.connect(lambda: self.pipeline.start_pipeline("minimal"))

        stop_btn = QPushButton("Stop Pipeline")
        stop_btn.clicked.connect(self.pipeline.stop)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self.rviz_container = QWidget()
        self.rviz_container.setMinimumSize(800, 600)
        self.rviz_container.setAttribute(Qt.WA_NativeWindow)
        self.rviz_container.setAttribute(Qt.WA_DontCreateNativeAncestors)
        self.rviz_container.winId()

        controls = QVBoxLayout()
        controls.addWidget(start_btn)
        controls.addWidget(stop_btn)
        controls.addWidget(self.log)

        main = QHBoxLayout()
        main.addLayout(controls, 1)
        main.addWidget(self.rviz_container, 3)

        central = QWidget()
        central.setLayout(main)
        self.setCentralWidget(central)

        self.start_rviz()

    def append_log(self, text):
        self.log.append(text)

    def start_rviz(self):
        self.rviz_process = QProcess(self)

        self.rviz_process.started.connect(
        lambda: print("[PY] rviz_widget_app STARTED")
        )
        self.rviz_process.errorOccurred.connect(
            lambda e: print("[PY] rviz_widget_app ERROR:", e)
        )
        self.rviz_process.readyReadStandardOutput.connect(
            lambda: print(
                "[RVIZ STDOUT]",
                bytes(self.rviz_process.readAllStandardOutput()).decode()
            )
        )
        self.rviz_process.readyReadStandardError.connect(
            lambda: print(
                "[RVIZ STDERR]",
                bytes(self.rviz_process.readAllStandardError()).decode()
            )
        )

        # X11 ID of the container widget
        xid = int(self.rviz_container.winId())

        env = os.environ.copy()
        env["QT_X11_NO_MITSHM"] = "1"
        env["QT_QPA_PLATFORM"] = "xcb"
        env["RVIZ_EMBED_WID"] = str(xid)

        self.rviz_process.setEnvironment([f"{key}={value}" for key, value in env.items()])

        rviz_config_path = os.path.abspath(
            "rviz/rqt_gui.rviz"
        )

        self.rviz_process.start(
            "/home/nexus/spot-teleop/rqt_interface/install/rviz_widget/lib/rviz_widget/rviz_widget_app"      
        )

