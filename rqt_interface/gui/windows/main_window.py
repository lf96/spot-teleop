from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout,
    QPushButton, QTextEdit
)
from controllers.pipeline_controller import PipelineController


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spot Teleop Control")
        self.resize(900, 600)

        self.pipeline = PipelineController(self)

        start_btn = QPushButton("Start Pipeline")
        start_btn.clicked.connect(lambda: self.pipeline.start_pipeline("full_stack"))

        stop_btn = QPushButton("Stop Pipeline")
        stop_btn.clicked.connect(self.pipeline.stop)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(start_btn)
        layout.addWidget(stop_btn)
        layout.addWidget(self.log)

        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

    def append_log(self, text):
        self.log.append(text)
