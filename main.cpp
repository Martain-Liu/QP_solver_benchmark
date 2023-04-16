#include<iostream>
#include<ctime>
#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include "Mpc_planner.h"
#include "common_data.h"
#include"Timer.h"

int main()
{
  //update_mpc_parameter
  float dt = 0.002;
  int horizon = 10;
  float mu = 0.5;
  float f_max = 1000;
  float dtMPC = 20*dt;

//update_dynamic_parameter
  FusionData fusion_data_;
  Eigen::Quaterniond quat( 0.999, -0.00521, -0.00901, 0.10479);
  Eigen::Matrix3d rot;rot = quat.toRotationMatrix();
  Eigen::Matrix<float, 4, 3> foot_position_in_body ;foot_position_in_body.setZero();
  foot_position_in_body <<0.2375, -0.077, -0.45, 0.2375, 0.077, -0.45, -0.2375, -0.077, -0.45, -0.2375, 0.077, -0.45; 
 fusion_data_.root_euler << -0.003008, -0.002459, -0.2096;
 fusion_data_.root_position <<0.0205, -0.016627, 0.448911;
 fusion_data_.root_angular_velocity_in_world<<-0.00984, -0.001845, 0.017129;
 fusion_data_.root_linear_velocity_in_world<<0.005269, 0.005931, -0.002371;
 fusion_data_.rotation_matrix_body_to_world = rot.cast<float>();
 fusion_data_.foot_position_in_body = foot_position_in_body;
OperatorData operator_data_;
operator_data_.root_linear_velocity_in_body<< 0.0, 0.0, 0.0;
operator_data_.root_angular_velocity_in_body <<0.0, 0.0, 0.0;

//update_ABmatrix_parameter
Eigen::Vector3f  inertial(1.65, 3.2, 3.02);
Eigen::Vector3f  com(0.0, 0.0, 0.0);
float mass = 60.0;

//
Eigen::Matrix<float, 12, 1> weights;
weights << 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1;
float alpha = 1e-5;
Mpc_planner planner;
planner.init();
planner.update_mpc_parameter(dt,  horizon, mu, f_max, dtMPC);
planner.update_dynamic_parameter(fusion_data_, operator_data_);
planner.update_ABmatrix_parameter(inertial, com, mass);
planner.update_qpOASES_parameter(weights, alpha);
for(int i=0; i<2;i++){
  Timer solveTimer;
  planner.run();
  double time = solveTimer.GetMs();
  std::cout<<"use time  == "<<time <<std::endl;
}


  

}
