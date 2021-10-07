#pragma once

#include <vector>
namespace vehicle_controller
{
enum WheelSide {left_front, right_front, right_rear, left_rear};
typedef double Double3[3];

typedef struct 
{
public:
    WheelSide wheel_side;
    Double3   direction;
    Double3   position;
} WheelData;

typedef struct 
{
public:
    Double3   direction;
    double    angular_velocity;
    std::vector<WheelData> wheel_data; 
} KinematicsData;
}