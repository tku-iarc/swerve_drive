#include <rclcpp/rclcpp.hpp>
// #include <signal.h>
#include "vehicle_controller/vehicle_controller.h"

using namespace vehicle_controller;

// void sigintHandler(int sig)
// {
//   // Do some custom action.
//   // For example, publish a stop message to some other nodes.
//   delete controller;
//   // All the default sigint handler does is call shutdown()
//   ros::shutdown();
// }

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// ros::AsyncSpinner spinner(1);
    // spinner.start();
	// signal(SIGINT, sigintHandler);
	std::string node_name("vehicle_controller");
	auto controller = std::make_shared<vehicle_controller::VehicleController>(node_name);
	rclcpp::spin(controller);
	// ros::Rate loop_rate(10);
	// while (ros::ok()){
	// 	ros::spinOnce();
	// 	// controller->process(loop_rate);
	// 	loop_rate.sleep();
	// }
	// spinner.stop();
	rclcpp::shutdown();
	return 0;
}