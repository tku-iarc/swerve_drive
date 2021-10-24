#include <ros/ros.h>
#include <signal.h>
#include "vehicle_controller/vehicle_controller.h"

using namespace vehicle_controller;

vehicle_controller::VehicleController* controller;

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
	ros::init(argc, argv, "vehicle_control", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	signal(SIGINT, sigintHandler);

	controller = new vehicle_controller::VehicleController(nodeHandle);

	ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		controller->process();
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}