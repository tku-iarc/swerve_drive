#pragma once
#include <vector>
#include <Eigen/Dense>
#include "vehicle_controller/vehicle_controller_define.h"

namespace vehicle_controller
{
class VehicleKinematics
{
public:
    VehicleKinematics();
    VehicleKinematics(KinematicsData &kinematics_data);
    ~VehicleKinematics();
    void configuration(KinematicsData &kinematics_data);
    bool forwardKinematics(KinematicsData &kinematics_data);
    bool inverseKinematics(KinematicsData &kinematics_data);
private:
    bool checkSlippage(KinematicsData &kinematics_data);
    void angularVelocityToDirection(const double &ang_vel, const Double2 &position, Double2 &direction);
    void matrixRemoveRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

    Eigen::MatrixXd mat_a_, mat_vehicle_, mat_wheels_;
};
} // namespace vehicle_controller