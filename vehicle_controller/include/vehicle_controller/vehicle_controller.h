#include <ros/ros.h>
#include "vehicle_controller/kinematics.h"

namespace vehicle_controller
{
class VehicleController
{
public:
    VehicleController();
    ~VehicleController();
    void process();
private:
    float dir_acc_max_;
    float ang_acc_max_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    ros::ServiceServer calib_server_;
    KinematicsData kinematics_data_;
};
}