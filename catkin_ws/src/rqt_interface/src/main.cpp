#include <QApplication>
#include "main_window.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rqt_interface");
  ros::start();
  QApplication app(argc, argv);

  MainWindow win;
  win.setWindowTitle("RQT Interface with Embedded RViz");
  win.resize(1200, 800);
  win.show();

  int ret = app.exec();

  ros::shutdown();

  return ret;
}
