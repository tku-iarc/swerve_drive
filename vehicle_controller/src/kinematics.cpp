#include <cstdio>
#include <iostream>
#include <cmath>
#include <numeric>
#include "vehicle_controller/kinematics.h"
namespace vehicle_controller
{
VehicleKinematics::VehicleKinematics()
{
}

VehicleKinematics::VehicleKinematics(KinematicsData &kinematics_data)
{
    this->configuration(kinematics_data);
}

VehicleKinematics::~VehicleKinematics()
{
}

void VehicleKinematics::configuration(KinematicsData &kinematics_data)
{
    size_t num_wheels = kinematics_data.wheel_data_vector.size();
    mat_a_.resize(num_wheels * 2, 3);
    mat_vehicle_.resize(3, 1);
    mat_wheels_.resize(num_wheels * 2, 1);
    // size_t i = 0;
    // for(auto it = kinematics_data.wheel_data.begin(); it != kinematics_data.wheel_data.end(); it++)
    for(Eigen::Index i = 0; i < mat_a_.rows(); i++)
    {
        mat_a_(i, 0) = (i + 1) % 2;
        mat_a_(i, 1) = i % 2;
        mat_a_(i, 2) = ((i % 2 == 0)? -1 : 1) * kinematics_data.wheel_data_vector[size_t(i / 2)]->pos_on_vehicle[(i + 1) % 2];
    }
}

bool VehicleKinematics::checkSlippage(KinematicsData &kinematics_data)
{
    double max_vbw = 0;
    std::string max_vbw_wheel;
    Eigen::Vector2d v_1, v_2, l_ij;
    for(auto it_fst = kinematics_data.wheel_data.begin(); it_fst != kinematics_data.wheel_data.end(); it_fst++)
    {
        double vbw = 0;
        it_fst->second->has_slippage = false;
        for(auto it_sec = kinematics_data.wheel_data.begin(); it_sec != kinematics_data.wheel_data.end(); it_sec++)
        {
            if(it_fst == it_sec)
                continue;
            v_1 << it_fst->second->direction[0],
                   it_fst->second->direction[1];
            v_2 << it_sec->second->direction[0],
                   it_sec->second->direction[1];
            l_ij << it_fst->second->pos_on_vehicle[0] - it_sec->second->pos_on_vehicle[0],
                    it_fst->second->pos_on_vehicle[1] - it_sec->second->pos_on_vehicle[1];
            l_ij /= l_ij.norm();
            vbw += fabs((v_1 - v_2).dot(l_ij));
        }
        if(vbw > max_vbw)
        {
            max_vbw = vbw;
            max_vbw_wheel = it_fst->first;
        }
    }
    kinematics_data.wheel_data[max_vbw_wheel]->has_slippage = true;
    return true;
}

bool VehicleKinematics::forwardKinematics(KinematicsData &kinematics_data)
{
    // double ang_vel = 0; double dir_x = 0; double dir_y = 0;
    // double ang_vel_cnt = 0; double dir_cnt = 0;

    // if(!checkSlippage(kinematics_data))
    // {
    //     return false;
    // }
    
    // for(auto it_fst=kinematics_data.wheel_data.begin(); it_fst!=(--kinematics_data.wheel_data.end()); it_fst++)
    // {
    //     if(it_fst->second->has_slippage)
    //         continue;

    //     auto it_fst_tmp = it_fst;
    //     for(auto it_sec=(++it_fst_tmp); it_sec!=kinematics_data.wheel_data.end(); it_sec++)
    //     {
    //         if(it_sec->second->has_slippage)
    //             continue;

    //         if(fabs(it_sec->second->pos_on_vehicle[1] - it_fst->second->pos_on_vehicle[1]) > 0.001)
    //         {
    //             ang_vel += (it_fst->second->direction[0] - it_sec->second->direction[0])
    //                     / (it_sec->second->pos_on_vehicle[1] - it_fst->second->pos_on_vehicle[1]);
    //             ang_vel_cnt+=1;
    //         }
    //         if(fabs(it_fst->second->pos_on_vehicle[0] - it_sec->second->pos_on_vehicle[0]) > 0.001)
    //         {
    //             ang_vel += (it_fst->second->direction[1] - it_sec->second->direction[1])
    //                     / (it_fst->second->pos_on_vehicle[0] - it_sec->second->pos_on_vehicle[0]);
    //             ang_vel_cnt += 1;
    //         }
    //     }
    // }

    // ang_vel /= ang_vel_cnt;

    // for(auto it=kinematics_data.wheel_data.begin(); it!=kinematics_data.wheel_data.end(); it++)
    // {
    //     if(it->second->has_slippage)
    //         continue;

    //     dir_x += it->second->direction[0] + ang_vel * it->second->pos_on_vehicle[1];
    //     dir_y += it->second->direction[1] - ang_vel * it->second->pos_on_vehicle[0];
    //     dir_cnt += 1;
    // }
    // kinematics_data.angular_velocity = ang_vel;
    // kinematics_data.direction[0] = dir_x / dir_cnt;
    // kinematics_data.direction[1] = dir_y / dir_cnt;

    Eigen::MatrixXd mat_a(mat_a_);
    Eigen::MatrixXd mat_wheels(mat_wheels_);
    for(size_t i = 0; i < kinematics_data.wheel_data_vector.size(); i++)
    {
        mat_wheels(i * 2, 0) = kinematics_data.wheel_data_vector[i]->direction[0];
        mat_wheels(i * 2 + 1, 0) = kinematics_data.wheel_data_vector[i]->direction[1];
    }
    for(size_t i = 0; i < kinematics_data.wheel_data_vector.size(); i++)
    {
        if(kinematics_data.wheel_data_vector[i]->has_slippage)
        {
            matrixRemoveRow(mat_a, i * 2);
            matrixRemoveRow(mat_a, i * 2 + 1);
            matrixRemoveRow(mat_wheels, i * 2);
            matrixRemoveRow(mat_wheels, i * 2 + 1);
        }
    }

    Eigen::MatrixXd mat_vehicle = (mat_a.transpose() * mat_a).ldlt().solve(mat_a.transpose() * mat_wheels);
    kinematics_data.direction[0] = mat_vehicle(0, 0);
    kinematics_data.direction[1] = mat_vehicle(1, 0);
    kinematics_data.angular_velocity = mat_vehicle(2, 0);

    return true;
}

bool VehicleKinematics::inverseKinematics(KinematicsData &kinematics_data)
{
    // for (auto it = kinematics_data.wheel_data.begin() ; it != kinematics_data.wheel_data.end(); ++it)
    // {
    //     Double2 wheel_direction;
    //     angularVelocityToDirection(kinematics_data.angular_velocity_cmd, it->second->pos_on_vehicle, wheel_direction);
    //     for(int i=0; i<2; i++)
    //         it->second->direction_cmd[i] = kinematics_data.direction_cmd[i] + wheel_direction[i];
    // }
    mat_vehicle_(0, 0) = kinematics_data.direction_cmd[0];
    mat_vehicle_(1, 0) = kinematics_data.direction_cmd[1];
    mat_vehicle_(2, 0) = kinematics_data.angular_velocity_cmd;
    mat_wheels_ = mat_a_ * mat_vehicle_;
    for(size_t i = 0; i < kinematics_data.wheel_data_vector.size(); i++)
    {
        kinematics_data.wheel_data_vector[i]->direction_cmd[0] = mat_wheels_(i * 2, 0);
        kinematics_data.wheel_data_vector[i]->direction_cmd[1] = mat_wheels_(i * 2 + 1, 0);
    }
    return true;
}

void VehicleKinematics::angularVelocityToDirection(const double &ang_vel, const Double2 &pos_on_vehicle, Double2 &direction)
{
    // position include the radius of rotation, so no need to calculate radius.
    // when ang_vel is positive, direction is exchange from position of wheel,
    // and trans fist item to negative, when ang_vel is negative, formula is the same.
    direction[0] = -1 * ang_vel * pos_on_vehicle[1];
    direction[1] = ang_vel * pos_on_vehicle[0];
}

void VehicleKinematics::matrixRemoveRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows, numCols);
}
}