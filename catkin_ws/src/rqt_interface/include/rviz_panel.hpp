#pragma once

#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <QWidget>
#include <QVBoxLayout>
#include <QApplication>



class RVizPanel : public QWidget
{
  Q_OBJECT

public:
  explicit RVizPanel(QWidget *parent = nullptr);
  ~RVizPanel();

public slots:
  void setupRViz();
  
private:
  rviz::VisualizationManager* manager_;
  rviz::VisualizationFrame* frame_;
  QVBoxLayout* layout_;
};