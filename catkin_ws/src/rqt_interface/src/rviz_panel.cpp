#include "rviz_panel.hpp"


RVizPanel::RVizPanel(QWidget *parent)
  : QWidget(parent)
  , frame_(new rviz::VisualizationFrame())
  , manager_(nullptr)
  , layout_(new QVBoxLayout(this))
{
  layout_->addWidget(frame_);
  layout_->setContentsMargins(0, 0, 0, 0);
  setLayout(layout_);
}
void RVizPanel::setupRViz()
{
ROS_INFO("[RVIZ_PANEL] Setting up RViz visualization Frame...");

  if(!frame_) {
    ROS_ERROR("[RVIZ_PANEL] Visualization frame is null.");
    return;
  }

  frame_->setSplashPath("");
  frame_->setMenuBar(nullptr);
  frame_->setStatusBar(nullptr);

  try
  {
    // Initialize the Visualization Frame
    frame_->initialize();

    // Create the Visualization Manager
    manager_ = frame_->getManager();
    if (!manager_) {
      ROS_ERROR("[RVIZ_PANEL] Failed to get Visualization Manager from frame.");
      return;
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("[RVIZ_PANEL] Failed to initialize RViz: %s", e.what());
      return;
  }

  ROS_INFO("[RVIZ_PANEL] RViz Visualization Frame startup complete.");
  
  manager_->setFixedFrame("body");
  
  QString config_path = QString::fromStdString(
    ros::package::getPath("rqt_interface") +
    "/rviz/rqt_gui.rviz");

  if(QFile::exists(config_path)) {
    try
    {
      frame_->loadDisplayConfig(config_path);
      ROS_INFO("[RVIZ_PANEL] Loaded RViz config from file: %s", config_path.toStdString().c_str());
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("[RVIZ_PANEL] Failed to load RViz config: %s", e.what());
    }
    
  } else {
    ROS_WARN("[RVIZ_PANEL] Default RViz config file not found at: %s", config_path.toStdString().c_str());
  }

  QApplication::processEvents();
  manager_->startUpdate();

}

RVizPanel::~RVizPanel() = default;