#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_
#include"eigen_types.h"

struct FusionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FusionData() { zero(); }

  void zero() {
    tick = 0.0;
    root_position = Vec3<float>::Zero();
    root_linear_velocity_in_world = Vec3<float>::Zero();
    root_linear_velocity_in_body = Vec3<float>::Zero();
    root_linear_velocity_in_yaw = Vec3<float>::Zero();
    root_acceleration_in_body = Vec3<float>::Zero();
    root_euler = Vec3<float>::Zero();
    root_quaternion << 1.0, 0.0, 0.0, 0.0;
    rotation_matrix_body_to_world = Mat3<float>::Zero();
    rotation_matrix_yaw_to_world = Mat3<float>::Zero();
    root_angular_velocity_in_world = Vec3<float>::Zero();
    root_angular_velocity_in_body = Vec3<float>::Zero();
    joint_position = Vec12<float>::Zero();
    joint_velocity = Vec12<float>::Zero();
    joint_acceleration = Vec12<float>::Zero();
    arm_joint_position = Vec6<float>::Zero();
    arm_joint_velocity = Vec6<float>::Zero();
    arm_joint_acceleration = Vec6<float>::Zero();
    torque_reading = Vec12<float>::Zero();
    foot_force_current = Vec12<float>::Zero();
    foot_contact_force = Vec4<float>::Zero();
    foot_contact_detected = Vec4<int>::Zero();
    foot_contact_probability = Vec4<float>::Zero();
    foot_position_in_body = Mat43<float>::Zero();
    foot_position_in_world = Mat43<float>::Zero();
    foot_position_in_relative = Mat43<float>::Zero();
    foot_velocity_in_body = Mat43<float>::Zero();
    foot_jacobian_in_body = Mat12<float>::Zero();
    foot_jacobian_derivative_in_body = Mat12<float>::Zero();
  }

  double tick;
  Vec3<float> root_position;
  Vec3<float> root_linear_velocity_in_world;
  Vec3<float> root_linear_velocity_in_body;
  Vec3<float> root_linear_velocity_in_yaw;
  Vec3<float> root_acceleration_in_body;
  Vec3<float> root_euler;
  Vec4<float> root_quaternion;
  Mat3<float> rotation_matrix_body_to_world;
  Mat3<float> rotation_matrix_yaw_to_world;
  Vec3<float> root_angular_velocity_in_world;
  Vec3<float> root_angular_velocity_in_body;
  Vec12<float> joint_position;
  Vec12<float> joint_velocity;
  Vec12<float> joint_acceleration;
  Vec6<float> arm_joint_position;
  Vec6<float> arm_joint_velocity;
  Vec6<float> arm_joint_acceleration;
  Vec12<float> torque_reading;
  Vec12<float> foot_force_current;
  Vec4<float> foot_contact_force;
  Vec4<int> foot_contact_detected;
  Vec4<float> foot_contact_probability;
  Mat43<float> foot_position_in_body;
  Mat43<float> foot_position_in_world;
  Mat43<float> foot_position_in_relative;
  Mat43<float> foot_velocity_in_body;
  Mat12<float> foot_jacobian_in_body;
  Mat12<float> foot_jacobian_derivative_in_body;
};

struct OperatorData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OperatorData() { zero(); }

  void zero() {
    cmd_src = 4;
    gait_mode = 1;
    run_state = 1;
    root_height = 0.0;
    brake_mode = 0;
    tracking_mode = 0;
    special_mode = 0;

    root_linear_velocity_in_body.setZero();
    root_angular_velocity_in_body.setZero();
    root_euler.setZero();
    root_position.setZero();
    root_orientation.setZero();
    root_acceleration.setZero();
    root_angular_acceleration.setZero();
  }

  uint8_t cmd_src;  // 1-ADS; 2-HRU; 3-RA; 4-GamePad;
  int32_t run_state;
  int8_t brake_mode;
  int8_t tracking_mode;
  uint8_t special_mode;
  int32_t gait_mode;
  float root_height;
  Vec3<float> root_euler;
  Vec3<float> root_position;
  Quat<float> root_orientation;
  Vec3<float> root_linear_velocity_in_body;
  Vec3<float> root_angular_velocity_in_body;
  Vec3<float> root_acceleration;
  Vec3<float> root_angular_acceleration;
};
#endif