#include <ros/ros.h>
#include <signal.h>
#include "wheel_controller/wheel_controller.h"
#include "maxon_epos2/epos_controller.hpp"

WheelController* controller;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  delete controller;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_control", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	signal(SIGINT, sigintHandler);
	std::string wheel_name = ros::this_node::getNamespace();
	controller = new WheelController(nodeHandle, wheel_name);

	ros::Rate loop_rate(controller->sample_rate);
	while (ros::ok()){
		ros::spinOnce();
		controller->process(loop_rate);
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}