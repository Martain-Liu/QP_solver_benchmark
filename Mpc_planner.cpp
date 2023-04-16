# include "Mpc_planner.h"
# include"eigen_types.h"
#include<iostream>
#include <unsupported/Eigen/MatrixFunctions>

Mpc_planner::Mpc_planner()
{
}

Mpc_planner::~Mpc_planner()
{
}

void Mpc_planner::init() { real_allocated = false; }

// NOTE：参数传递分为两部分：1）用于构建x_dot = A*x+Bu的参数；2）用于设置MPC的参数
void Mpc_planner::update_mpc_parameter(float& dt, int& horizon, float& mu, float& f_max, float& dtMPC) {
  dt_ = dt;
  dtMPC_ = dtMPC;
  horizon_ = horizon;
  mu_ = mu;
  f_max_ = f_max;
}

void Mpc_planner::update_dynamic_parameter(
    const FusionData& fusion_data, const OperatorData& operator_data) {
  // NOTE：初始轨迹x_0所需参数
  root_euler_ = fusion_data.root_euler;
  root_position_ = fusion_data.root_position;
  root_angular_velocity_in_world_ = fusion_data.root_angular_velocity_in_world;
  root_linear_velocity_in_world_ = fusion_data.root_linear_velocity_in_world;
  rotation_matrix_body_to_world_ = fusion_data.rotation_matrix_body_to_world;
  foot_position_in_body_ = fusion_data.foot_position_in_body;

  float yc = cos(root_euler_[2]);
  float ys = sin(root_euler_[2]);
  R_yaw_ << yc, -ys, 0, ys, yc, 0, 0, 0, 1;

  // NOTE: 初始状态
  x_0_ << fusion_data.root_euler, fusion_data.root_position, fusion_data.root_angular_velocity_in_world,
      fusion_data.root_linear_velocity_in_world, -9.8;

  // NOTE：参考轨迹
  float yaw_des;
  float yaw_turn_rate;
  Eigen::Matrix3f root_mat_z = coordinateRotation(CoordinateAxis::Z, fusion_data.root_euler[2]);
  Eigen::Vector3f v_des_robot;
  v_des_robot.setZero();
  Eigen::Vector3f v_des_world;
  v_des_world.setZero();
  Eigen::Vector3f world_position_desired;
  world_position_desired.setZero();
  if (first_run_) {
    // v_des_world << 0.0, 0.0, 0.0;
    world_position_desired << fusion_data.root_position(0), fusion_data.root_position(1), 0.0;
    yaw_des = (fusion_data.rotation_matrix_body_to_world * root_euler_)(2);
    first_run_ = false;
  }
  yaw_turn_rate = 50 * operator_data.root_angular_velocity_in_body[2];
  yaw_des += dt_ * yaw_turn_rate;
  v_des_robot = 50 * operator_data.root_linear_velocity_in_body;
  v_des_world = root_mat_z.transpose() * v_des_robot;
  world_position_desired += dt_ * v_des_world;
  traj_init_ << 0.0, 0.0, yaw_des, world_position_desired[0], world_position_desired[1], 0.45, 0.0, 0.0, yaw_turn_rate,
      v_des_world[0], v_des_world[1], 0.0;

  for (int i = 0; i < HORIZON_LENGTH; i++) {
    traj_all_.block(13 * i, 0, 12, 1) = traj_init_;
    if (i > 0) {
      traj_all_(13 * i + 2, 0) = traj_all_(13 * (i - 1) + 2, 0) + dtMPC_ * yaw_turn_rate;
      traj_all_(13 * i + 3, 0) = traj_all_(13 * (i - 1) + 3, 0) + dtMPC_ * v_des_world[0];
      traj_all_(13 * i + 4, 0) = traj_all_(13 * (i - 1) + 4, 0) + dtMPC_ * v_des_world[1];
    }
  }
}

void Mpc_planner::update_ABmatrix_parameter(Eigen::Vector3f& inertial, Eigen::Vector3f& com, float mass) {
  I_body_ = inertial;
  mass_ = mass;
  com_in_body_ = com;

  I_world_ = R_yaw_ * I_body_.asDiagonal() * R_yaw_.transpose();
  I_world_inv_ = I_world_.inverse();
}

void Mpc_planner::update_qpOASES_parameter(Eigen::Matrix<float, 12, 1>& weights, float alpha) {
  // NOTE : 权重参数
  weights_ << weights, 0.0;
  S_.diagonal() = weights_.replicate<HORIZON_LENGTH, 1>();
  alpha_ = alpha;
}

void Mpc_planner::run() {
  // NOTE:：构建A B矩阵
  A_.block<3, 3>(0, 6) = R_yaw_;
  A_.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();
  A_(11, 12) = 1.0;

  for (int leg = 0; leg < 4; leg++) {
    r_com_to_foot_ = rotation_matrix_body_to_world_ * (foot_position_in_body_.transpose().col(leg) - com_in_body_);
    B_.block<3, 3>(6, 3 * leg) = cross_mat(I_world_inv_, r_com_to_foot_);
    B_.block<3, 3>(9, 3 * leg) = Eigen::Matrix<float, 3, 3>::Identity() / mass_;
  }
//   std::cout<<"A:\n" <<A_ <<std::endl;
//   std::cout<<"B:\n" <<B_ <<std::endl;
  // NOTE：构建A_qp<13*10, 13>，B_qp<13*10, 12*10>矩阵
  Eigen::Matrix<float, 25, 25> ABc, expmm;
  Eigen::Matrix<float, 13, 13> Adt;
  Eigen::Matrix<float, 13, 12> Bdt;
  Eigen::Matrix<float, 12 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> eye12h;
  Eigen::Matrix<float, 5, 3> f_block;
  Eigen::Matrix<float, 5, 1> ubA_submatrix;
  ABc.setZero();
  Adt.setZero();
  Bdt.setZero();
  eye12h.setIdentity();
  expmm.setZero();
  ABc.block<13, 13>(0, 0) = A_;
  ABc.block<13, 12>(0, 13) = B_;
  ABc = dt_ * ABc;
  expmm = ABc.exp();
  Adt = expmm.block<13, 13>(0, 0);
  Bdt = expmm.block<13, 12>(0, 13);

  Eigen::Matrix<float, 13, 13> powerMats[100];
  powerMats[0].setIdentity();
  for (int i = 1; i < HORIZON_LENGTH + 1; i++) { powerMats[i] = Adt * powerMats[i - 1]; }
  for (int r = 0; r < HORIZON_LENGTH; r++) {

    A_qp_.block(13 * r, 0, 13, 13) = powerMats[r + 1];
    for (int n = 0; n < HORIZON_LENGTH; n++) {
      if (n <= r) { 
          B_qp_.block(13 * r, 12 * (r - n), 13, 12) = powerMats[n] * Bdt; 

      }
    }
  }

  // NOTE：构建qp求解器标准形式   -    H g  A  lb ub lbA ubA
  f_block << 1.f / mu_, 0, 1.f, -1.f / mu_, 0, 1.f, 0, 1.f / mu_, 1.f, 0, -1.f / mu_, 1.f, 0, 0, 1.f;

  for (int i = 0; i < 4 * HORIZON_LENGTH; i++) { A_matrix_.block(5 * i, 3 * i, 5, 3) = f_block; }

  ubA_submatrix << BIG_NUMBER, BIG_NUMBER, BIG_NUMBER, BIG_NUMBER, f_max_;
  ubA_matrix_ = ubA_submatrix.replicate<4 * HORIZON_LENGTH, 1>();

  lbA_matrix_.setZero();

  H_matrix_ = B_qp_.transpose() * S_ * B_qp_ + alpha_ * eye12h;
  g_matrix_ = B_qp_.transpose() * S_ * (A_qp_ * x_0_ - traj_all_);

  // NOTE：构建qp求解器
  int new_vars = 12 * HORIZON_LENGTH;
  int new_cons = 20 * HORIZON_LENGTH;

  if (real_allocated) {
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_red = (qpOASES::real_t*)malloc(12 * HORIZON_LENGTH * 12 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  g_red = (qpOASES::real_t*)malloc(12 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  A_red = (qpOASES::real_t*)malloc(20 * HORIZON_LENGTH * 12 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  lb_red = (qpOASES::real_t*)malloc(20 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  ub_red = (qpOASES::real_t*)malloc(20 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  q_red = (qpOASES::real_t*)malloc(12 * HORIZON_LENGTH * sizeof(qpOASES::real_t));
  Eigen2real_t(H_matrix_, H_red, 12 * HORIZON_LENGTH, 12 * HORIZON_LENGTH);
  Eigen2real_t(g_matrix_, g_red, 12 * HORIZON_LENGTH, 1);
  Eigen2real_t(A_matrix_, A_red, 20 * HORIZON_LENGTH, 12 * HORIZON_LENGTH);
  Eigen2real_t(ubA_matrix_, ub_red, 20 * HORIZON_LENGTH, 1);
  Eigen2real_t(lbA_matrix_, lb_red, 20 * HORIZON_LENGTH, 1);

  real_allocated = true;
  qpOASES::QProblem problem_red(new_vars, new_cons);
  qpOASES::Options op;
  op.setToMPC();
//   op.setToReliable();
  op.printLevel = qpOASES::PL_DEBUG_ITER;
  problem_red.setOptions(op);
  qpOASES::real_t cpu_time{0.01};
  qpOASES::int_t nWSR = 1000;
  int rval = problem_red.init(H_red, g_red, NULL, NULL, NULL, NULL, NULL, nWSR);
 int rval2 = problem_red.getPrimalSolution(q_red);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN) printf("failed to solve!\n");
}

void Mpc_planner::get_result() {}

Eigen::Matrix<float, 3, 3> Mpc_planner::cross_mat(Eigen::Matrix<float, 3, 3> I_inv, Eigen::Matrix<float, 3, 1> r) {
  Eigen::Matrix<float, 3, 3> cm;
  cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;
  return I_inv * cm;
}

template <typename Derived>
void Mpc_planner::Eigen2real_t(const Eigen::MatrixBase<Derived>& mat_in, qpOASES::real_t* mat_out, int rows, int cols) {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) { mat_out[i * cols + j] = mat_in(i, j); }
  }
}

Eigen::Matrix3f  Mpc_planner::coordinateRotation(CoordinateAxis axis, float theta) {
//   static_assert(std::is_floating_point<T>::value, "must use floating point value");
  float s = std::sin(theta);
  float c = std::cos(theta);

  Eigen::Matrix<float, 3, 3> R;

  if (axis == CoordinateAxis::X) {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}