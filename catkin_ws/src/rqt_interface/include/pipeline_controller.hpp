#pragma once

#include <string>

class PipelineController {
public:
  explicit PipelineController(const std::string& socket_path);
  void runPipeline(const std::string& pipeline_name);
  void stop();
  void openGripper();
  void closeGripper();

private:
  void requestHostExecution(const std::string& command);
  std::string socket_path_;
};
