#include <ros/ros.h>
#include <signal.h>
#include "wheel_controller/wheel_controller.h"
#include "maxon_epos2/epos_controller.hpp"

WheelController* wheel_controller;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
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
	std::cout<<"qqq"<<std::endl;

	wheel_controller = new WheelController(nodeHandle);

	// std::cout<<"wheel_controller->sample_rate"<<std::endl;
	ros::Rate loop_rate(wheel_controller->sample_rate);
	while (ros::ok()){
		ros::Time start_time = ros::Time::now();
		ros::spinOnce();
		wheel_controller->process(loop_rate);
		loop_rate.sleep();
		ros::Time end_time = ros::Time::now();
		// std::cout<<"loop time = "<<(end_time - start_time).toSec() * 1000 << " ms"<<std::endl;
	}
	spinner.stop();
	return 0;
}