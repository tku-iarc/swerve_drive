#include <cmath>
#include "vehicle_controller/kinematics.h"
namespace vehicle_controller
{
VehicleKinematics::VehicleKinematics()
{
}
VehicleKinematics::~VehicleKinematics()
{
}
bool VehicleKinematics::forwardKinematics(KinematicsData &kinematics_data)
{

}
bool VehicleKinematics::inverseKinematics(KinematicsData &kinematics_data)
{
    for (auto it = kinematics_data.wheel_data.begin() ; it != kinematics_data.wheel_data.end(); ++it)
    {
        Double2 wheel_direction;
        angularVelocityToDirection(kinematics_data.angular_velocity, it->position, wheel_direction);
        for(int i=0; i<2; i++)
            it->direction[i] = kinematics_data.direction[i] + wheel_direction[i];
    }
}
void VehicleKinematics::angularVelocityToDirection(const double &ang_vel, const Double2 &position, Double2 &direction)
{
    // position include the radius of rotation, so no need to calculate radius.
    // when ang_vel is positive, direction is exchange from position of wheel,
    // and trans fist item to negative, when ang_vel is negative, formula is the same.
    direction[0] = -1 * ang_vel * position[1];
    direction[1] = ang_vel * position[0];
}

double VehicleKinematics::metersToRads(const double &meters)
{
    return meters / WHEEL_RADIUS;
}
double VehicleKinematics::radsTometers(const double &rads)
{
    return rads * WHEEL_RADIUS;
}
}