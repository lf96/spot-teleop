#include "pipeline_controller.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cerrno>

PipelineController::PipelineController(const std::string& socket_path)
: socket_path_(socket_path)
{
  if (access(socket_path_.c_str(), F_OK) != 0) {
    std::cerr << "[PIPELINE] Socket path does not exist: " << socket_path_ << std::endl;
  }
}

void PipelineController::runPipeline(const std::string& pipeline_name)
{

  std::cout << "[PIPELINE] Running pipeline: " << pipeline_name << std::endl;
  requestHostExecution(pipeline_name);

}

void PipelineController::stop()
{
  std::cout << "[PIPELINE] Stopping pipeline" << std::endl;
  requestHostExecution("stop");
}

void PipelineController::openGripper()
{
  std::cout << "[PIPELINE] Opening gripper" << std::endl;
  requestHostExecution("open_gripper");
}

void PipelineController::closeGripper()
{
  std::cout << "[PIPELINE] Closing gripper" << std::endl;
  requestHostExecution("close_gripper");
}

void PipelineController::requestHostExecution(const std::string& command)
{
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    perror("socket");
    return;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (connect(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("connect");
    close(fd);
    return;
  }

  write(fd, command.c_str(), command.size());

  char buffer[64]{};
  read(fd, buffer, sizeof(buffer));
  std::cout << "[HOST] " << buffer << std::endl;

  close(fd);
}
