#include <ros/ros.h>
#include <signal.h>
#include "wheel_controller/wheel_controller.h"

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
	controller = new WheelController(nodeHandle);

	ros::Rate loop_rate(controller->sample_rate);
	while (ros::ok()){
		ros::spinOnce();
		controller->process(loop_rate);
    	controller->statePublish();
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}