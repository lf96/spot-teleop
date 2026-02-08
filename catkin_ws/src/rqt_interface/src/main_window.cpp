#include "main_window.hpp"

MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent)
{ 
  // Set up RViz panel
  rviz_panel_ = new RVizPanel(this);

  // Set up Pipeline controls
  pipeline_controller_ = std::make_unique<PipelineController>("/home/ipc/pipeline_controller.sock");

  // Setup UI elements
  log = new QTextEdit();
  log->setReadOnly(true);
  // connect(
  //   pipeline_controller_,
  //   &PipelineController::log,
  //   this,
  //   [this](const QString& text) {
  //     log->append(text);
  //   });

  start_btn = new QPushButton("Start Pipeline");
  start_btn->connect(start_btn, &QPushButton::clicked, [this]() {
    log->append("Start button clicked");
    pipeline_controller_->runPipeline("minimal");
  });
  stop_btn  = new QPushButton("Stop Pipeline");
  stop_btn->connect(stop_btn, &QPushButton::clicked, [this]() {
    log->append("Stop button clicked");
    pipeline_controller_->stop();
  });
  
  // Controls layout
  controls_layout = new QVBoxLayout();

  controls_layout->addWidget(start_btn);
  controls_layout->addWidget(stop_btn);
  controls_layout->addWidget(log);
  controls_layout->addStretch();

  // Main layout
  main_layout = new QHBoxLayout();
  main_layout->addLayout(controls_layout, 1);
  main_layout->addWidget(rviz_panel_, 4);
  auto central_widget = new QWidget();
  central_widget->setLayout(main_layout);
  setCentralWidget(central_widget);

  QTimer::singleShot(0, rviz_panel_, SLOT(setupRViz()));

}

MainWindow::~MainWindow() = default;


