#pragma once
#include <vector>

#define WHEEL_DIAMETER 0.15
#define WHEEL_RADIUS 0.075

namespace vehicle_controller
{
enum WheelSide {left_front, right_front, right_rear, left_rear};
typedef double Double2[2];

typedef struct 
{
public:
    WheelSide wheel_side;
    Double2   direction;
    Double2   position;
} WheelData;

typedef struct 
{
public:
    Double2   direction;
    double    angular_velocity;
    std::vector<WheelData> wheel_data; 
} KinematicsData;
}