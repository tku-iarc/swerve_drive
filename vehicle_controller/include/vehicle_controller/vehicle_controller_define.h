#pragma once
#include <vector>
#include <string>
#include <map>

namespace vehicle_controller
{
typedef double Double2[2];

typedef struct 
{
public:
    std::vector<std::string> joints_name;
    std::string wheel_name;
    Double2     direction;
    Double2     direction_cmd;
    Double2     pos_on_vehicle;
    bool        has_slippage;
} WheelData;

typedef struct 
{
public:
    Double2   position;
    Double2   direction;
    Double2   direction_cmd;
    double    rotation;
    double    angular_velocity_cmd;
    double    angular_velocity;
    std::vector<WheelData*> wheel_data_vector;
    std::map<std::string, WheelData*> wheel_data;
} KinematicsData;
}