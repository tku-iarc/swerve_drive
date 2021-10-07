#include <vector>
#include "vehicle_controller/vehicle_controller_define.h"

namespace vehicle_controller
{
class VehicleKinematics
{
public:
    VehicleKinematics();
    ~VehicleKinematics();
    bool forwardKinematics(KinematicsData &kinematics_data);
    bool inverseKinematics(KinematicsData &kinematics_data);
private:
    double metersToRads(const double &meters);
    double radsTometers(const double &rads);
}
} // namespace vehicle_controller