#pragma once

#include <QTimer>

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include "rviz_panel.hpp"
#include "pipeline_controller.hpp"
#include <memory>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  RVizPanel* rviz_panel_;
  std::unique_ptr<PipelineController> pipeline_controller_;
  rviz::VisualizationFrame* rviz_frame_;
  QPushButton* start_btn;
  QPushButton* stop_btn;
  QPushButton* reset_view_btn;
  QPushButton* open_gripper_btn;
  QPushButton* close_gripper_btn;
  QTextEdit* log;
  QVBoxLayout* controls_layout;
  QHBoxLayout* main_layout;
  QHBoxLayout* gripper_layout;

};