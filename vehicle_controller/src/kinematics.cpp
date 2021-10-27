#include <cstdio>
#include <iostream>
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
    double ang_vel = 0; double dir_x = 0; double dir_y = 0;
    double ang_vel_cnt = 0; double dir_cnt = 0;
    // for(int i=0; i<kinematics_data.wheel_data.size()-1; i++)
    // {
    //     for(int j=i+1; j<kinematics_data.wheel_data.size(); j++)
    //     {
    //         ang_vel += (kinematics_data.wheel_data[i].direction[0] - kinematics_data.wheel_data[j].direction[0])
    //                    / (kinematics_data.wheel_data[j].pos_on_vehicle[1] - kinematics_data.wheel_data[i].pos_on_vehicle[1]);
    //         ang_vel += (kinematics_data.wheel_data[i].direction[1] - kinematics_data.wheel_data[j].direction[1])
    //                    / (kinematics_data.wheel_data[i].pos_on_vehicle[0] - kinematics_data.wheel_data[j].pos_on_vehicle[0]);
    //         ang_vel_cnt += 2;
    //     }
    // }
    for(auto it_fst=kinematics_data.wheel_data.begin(); it_fst!=(--kinematics_data.wheel_data.end()); it_fst++)
    {
        auto it_fst_tmp = it_fst;
        for(auto it_sec=(++it_fst_tmp); it_sec!=kinematics_data.wheel_data.end(); it_sec++)
        {
            if(fabs(it_sec->second.pos_on_vehicle[1] - it_fst->second.pos_on_vehicle[1]) > 0.001)
            {
                ang_vel += (it_fst->second.direction[0] - it_sec->second.direction[0])
                        / (it_sec->second.pos_on_vehicle[1] - it_fst->second.pos_on_vehicle[1]);
                ang_vel_cnt+=1;
            }
            if(fabs(it_fst->second.pos_on_vehicle[0] - it_sec->second.pos_on_vehicle[0]) > 0.001)
            {
                ang_vel += (it_fst->second.direction[1] - it_sec->second.direction[1])
                        / (it_fst->second.pos_on_vehicle[0] - it_sec->second.pos_on_vehicle[0]);
                ang_vel_cnt += 1;
            }
        }
    }

    ang_vel /= ang_vel_cnt;
    // for(int i=0; i<kinematics_data.wheel_data.size(); i++)
    // {
    //     dir_x += kinematics_data.wheel_data[i].direction[0] + ang_vel * kinematics_data.wheel_data[i].pos_on_vehicle[1];
    //     dir_y += kinematics_data.wheel_data[i].direction[1] + ang_vel * kinematics_data.wheel_data[i].pos_on_vehicle[0];
    //     dir_cnt += 1;
    // }
    for(auto it=kinematics_data.wheel_data.begin(); it!=kinematics_data.wheel_data.end(); it++)
    {
        dir_x += it->second.direction[0] + ang_vel * it->second.pos_on_vehicle[1];
        dir_y += it->second.direction[1] - ang_vel * it->second.pos_on_vehicle[0];
        dir_cnt += 1;
    }
    kinematics_data.angular_velocity = ang_vel;
    kinematics_data.direction[0] = dir_x / dir_cnt;
    kinematics_data.direction[1] = dir_y / dir_cnt;
}

bool VehicleKinematics::inverseKinematics(KinematicsData &kinematics_data)
{
    for (auto it = kinematics_data.wheel_data.begin() ; it != kinematics_data.wheel_data.end(); ++it)
    {
        Double2 wheel_direction;
        angularVelocityToDirection(kinematics_data.angular_velocity_cmd, it->second.pos_on_vehicle, wheel_direction);
        for(int i=0; i<2; i++)
            it->second.direction_cmd[i] = kinematics_data.direction_cmd[i] + wheel_direction[i];
    }
}

void VehicleKinematics::angularVelocityToDirection(const double &ang_vel, const Double2 &pos_on_vehicle, Double2 &direction)
{
    // position include the radius of rotation, so no need to calculate radius.
    // when ang_vel is positive, direction is exchange from position of wheel,
    // and trans fist item to negative, when ang_vel is negative, formula is the same.
    direction[0] = -1 * ang_vel * pos_on_vehicle[1];
    direction[1] = ang_vel * pos_on_vehicle[0];
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