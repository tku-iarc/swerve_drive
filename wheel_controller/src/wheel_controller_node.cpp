#include <ros/ros.h>
#include <signal.h>
#include "wheel_controller/wheel_controller.h"
#include "maxon_epos2/EposController.hpp"

WheelController* blue_arm;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  blue_arm->closeDevice();
  delete blue_arm;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "blue_arm_control", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	signal(SIGINT, sigintHandler);

	blue_arm = new WheelController(nodeHandle);

	ros::Rate loop_rate(blue_arm->sample_rate);
	while (ros::ok()){
		blue_arm->process(loop_rate);
		ros::spinOnce();
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}