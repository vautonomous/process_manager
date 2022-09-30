#include "ui_process_manager/ui_process_manager.hpp"

#include <csignal>
#include <iostream>
#include <functional>

const std::string name_node = "ui_process_manager";

std::function<void()> callback_kill_cleanup;

void SigHandler(int sig) {
  std::cerr << name_node + " died with #" + std::to_string(sig) << " - " << strsignal(sig) << std::endl;

  if(callback_kill_cleanup) {
    callback_kill_cleanup();
  }

  exit(0);
}

int main(int argc, char ** argv)
{
  std::vector<int> signals{
    SIGINT,
    SIGILL,
    SIGABRT,
    SIGFPE,
    SIGSEGV,
    SIGTERM,
    SIGHUP,
    SIGQUIT,
    SIGTRAP,
    SIGKILL,
    SIGBUS,
    SIGSYS,
    SIGPIPE,
    SIGALRM,
    SIGURG,
    SIGSTOP,
    SIGTSTP,
    SIGCONT,
    SIGCHLD,
    SIGTTIN,
    SIGTTOU,
    SIGPOLL,
    SIGXCPU,
    SIGXFSZ,
    SIGVTALRM,
    SIGPROF
  };

  for (const auto &sig : signals) {
    signal(sig, SigHandler);
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<UIProcessManager>(name_node + "_node", node_options);

  callback_kill_cleanup = [&node](){
    node->killAutoware();
  };

  rclcpp::spin(node);
  return 0;
}
