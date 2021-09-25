#include <ros/ros.h>
#include <signal.h>
#include "wheel_controller/wheel_controller.h"
#include "maxon_epos2/EposController.hpp"

WheelController* wheel_controller;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  wheel_controller->closeDevice();
  delete wheel_controller;
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

	wheel_controller = new WheelController(nodeHandle);

	ros::Rate loop_rate(wheel_controller->sample_rate);
	while (ros::ok()){
		wheel_controller->process(loop_rate);
		ros::spinOnce();
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}