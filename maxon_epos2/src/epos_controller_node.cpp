#include <signal.h>
#include "maxon_epos2/epos_controller.hpp"

maxon_epos2::EposController* epos_controller;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  epos_controller->closeDevice();
  delete epos_controller;
  // All the default sigint handler does is call shutdown()
  ROS_INFO("maxon_epos2 shut down");
  ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "maxon_epos2", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	signal(SIGINT, sigintHandler);
    int num_motors = 0;
    std::vector<int> id_list;
    std::vector<int> pos_list;
    std::vector<int> vel_list;
    nodeHandle.getParam("id_list", id_list);
	nodeHandle.getParam("pos_list", pos_list);
    nodeHandle.getParam("vel_list", vel_list);

	epos_controller = new maxon_epos2::EposController(nodeHandle);
	while (ros::ok()){
		ros::spinOnce();
        // ros::Time begin = ros::Time::now();
        if(epos_controller->deviceOpenedCheck() == false)
            continue;
        for(auto it = pos_list.begin(); it != pos_list.end(); ++it)
            epos_controller->writePosition(*it);
        for(auto it = pos_list.begin(); it != pos_list.end(); ++it)
        {
            epos_controller->readPosition(*it);
            epos_controller->readVelocity(*it);
        }
        for(int i=0; i<vel_list.size(); i++)
        {
            double cmd = epos_controller->getCmd(vel_list[i]) - (epos_controller->getVel(pos_list[i]) * 0.1566416);
            epos_controller->writeVelocity(vel_list[i], cmd);
        }
        for(auto it = vel_list.begin(); it != vel_list.end(); ++it)
            epos_controller->readVelocity(*it);
        epos_controller->motorStatesPublisher();
        // ros::Time end = ros::Time::now();
        // std::cout<<"while time = "<<(end - begin).toSec() * 1000<<std::endl;
	}
	spinner.stop();
	return 0;
}