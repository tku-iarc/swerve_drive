#include <signal.h>
#include <rclcpp/init_options.hpp>
#include "maxon_epos2/epos_controller.hpp"

// void sigintHandler(int sig)
// {
//   // Do some custom action.
//   // For example, publish a stop message to some other nodes.
//   epos_controller->closeDevice();
//   delete epos_controller;
  
//   rclcpp::shutdown();
// }

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// rclcpp::AsyncSpinner spinner(1);
    // spinner.start();
	// signal(SIGINT, sigintHandler);
    std::string node_name("maxon_epos2");
    auto epos_controller = std::make_shared<maxon_epos2::EposController>(node_name);
    rclcpp::spin(epos_controller);
	// spinner.stop();
    epos_controller->closeDevice();
    rclcpp::shutdown();
	return 0;
}