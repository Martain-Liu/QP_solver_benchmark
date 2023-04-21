#include "Mpc_planner_OSQPEigen.h"
Mpc_planner_OSQPEigen::Mpc_planner_OSQPEigen() {}

Mpc_planner_OSQPEigen::~Mpc_planner_OSQPEigen() {}

void Mpc_planner_OSQPEigen::init() {
  first_run_ = true;
  Q_weights.setZero();
  R_weights.setZero();
  Q_full_weights.setZero();
  R_full_weights.setZero();
  hessian.resize(13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH, 13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH);
  hessian.setZero();
  gradient.resize(13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH, 1);
  gradient.setZero();
  linearMatrix.resize(
      13 * (HORIZON_LENGTH + 1) + 13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH,
      13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH);
  linearMatrix.setZero();
  upperBound.resize(13 * (HORIZON_LENGTH + 1) + 13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH, 1);
  upperBound.setZero();
  lowerBound.resize(13 * (HORIZON_LENGTH + 1) + 13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH, 1);
  lowerBound.setZero();
  state_max.resize(13);
  state_max.setZero();
  state_min.resize(13);
  state_min.setZero();
  input_max.resize(12);
  input_max.setZero();
  input_min.resize(12);
  input_min.setZero();
}

void Mpc_planner_OSQPEigen::set_solver_parameter(
    Eigen::Matrix<double, 12, 1>& Q,
    Eigen::Matrix<double, 12, 1>& R,
    Eigen::Matrix<double, 12, 1>& statemax,
    Eigen::Matrix<double, 12, 1>& statemin,
    Eigen::Matrix<double, 12, 1>& inputmax,
    Eigen::Matrix<double, 12, 1>& inputmin) {
  // TODO：更新求解器所需参数
  Q_weights << Q, 0.0;
  R_weights << R, 0.0;
  state_max << statemax, 0.0;
  state_min << statemin, 0.0;
  input_max = inputmax;
  input_min = inputmin;
}

void Mpc_planner_OSQPEigen::set_state_parameter(const FusionData& fusion_data, const OperatorData& operator_data) {
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

  for (int i = 0; i < (HORIZON_LENGTH + 1); i++) {
    traj_all_.block(13 * i, 0, 12, 1) = traj_init_;
    if (i > 0) {
      traj_all_(13 * i + 2, 0) = traj_all_(13 * (i - 1) + 2, 0) + dtMPC_ * yaw_turn_rate;
      traj_all_(13 * i + 3, 0) = traj_all_(13 * (i - 1) + 3, 0) + dtMPC_ * v_des_world[0];
      traj_all_(13 * i + 4, 0) = traj_all_(13 * (i - 1) + 4, 0) + dtMPC_ * v_des_world[1];
    }
  }
}

void Mpc_planner_OSQPEigen::castMPCToQPHessian() {
  // populate hessian matrix
  Q_full_weights = Q_weights.replicate<(HORIZON_LENGTH + 1), 1>();
  R_full_weights = R_weights.replicate<HORIZON_LENGTH, 1>();
  hessian.diagonal() << Q_full_weights, R_full_weights;
}

void Mpc_planner_OSQPEigen::castMPCToQPGradient() {
  Eigen::Matrix<double, 13 * (HORIZON_LENGTH + 1), 1> Q_trajall;
  Q_trajall = Q_full_weights.asDiagonal() * traj_all_.cast<double>();
  gradient.head(13 * (HORIZON_LENGTH + 1)) = Q_trajall.cast<double>();
}

void Mpc_planner_OSQPEigen::castMPCToQPConstraintMatrix() {
  // NOTE:：构建A B矩阵
  A_.block<3, 3>(0, 6) = R_yaw_;
  A_.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();
  A_(11, 12) = 1.0;

  for (int leg = 0; leg < 4; leg++) {
    r_com_to_foot_ = rotation_matrix_body_to_world_ * (foot_position_in_body_.transpose().col(leg) - com_in_body_);
    B_.block<3, 3>(6, 3 * leg) = cross_mat(I_world_inv_, r_com_to_foot_);
    B_.block<3, 3>(9, 3 * leg) = Eigen::Matrix<float, 3, 3>::Identity() / mass_;
  }
  for (int i = 0; i < 13 * (HORIZON_LENGTH + 1); ++i) { linearMatrix.insert(i, i) = -1; }
  for (int j = 0; j < (13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH); ++j) {
    linearMatrix.insert(13 * (HORIZON_LENGTH + 1) + j, j) = 1;
  }
  for (int m = 0; m < HORIZON_LENGTH; ++m) {
    for (int i = 0; i < 13; ++i) {
      for (int j = 0; j < 13; ++j) { linearMatrix.insert(13 + 13 * m + i, 13 * m + j) = A_(i, j); }
    }
    // linearMatrix.block(13 + 13 * m, 13 * m, 13, 13) = A_; }
    for (int n = 0; n < HORIZON_LENGTH; ++n) {
      for (int i = 0; i < 13; ++i) {
        for (int j = 0; j < 12; ++j) {
          linearMatrix.insert(13 + 13 * n + i, 13 * (HORIZON_LENGTH + 1) + 12 * n + j) = B_(i, j);
        }
      }
      // linearMatrix.block(13 + 13 * n, 13 * (HORIZON_LENGTH + 1) + 12 * n, 13, 12) = B_;
    }
  }
}
void Mpc_planner_OSQPEigen::castMPCToQPConstraintVectors() {
  Eigen::VectorXd lower_equality(13 * (HORIZON_LENGTH + 1));
  lower_equality.setZero();
  Eigen::VectorXd upper_equality(13 * (HORIZON_LENGTH + 1));
  upper_equality.setZero();
  lower_equality.segment(0, 13) = -x_0_.cast<double>();
  upper_equality.segment(0, 13) = -x_0_.cast<double>();

  Eigen::VectorXd lower_inequality(13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH);
  Eigen::VectorXd upper_inequality(13 * (HORIZON_LENGTH + 1) + 12 * HORIZON_LENGTH);
  Eigen::VectorXd state_max_inquality(13 * (HORIZON_LENGTH + 1));
  Eigen::VectorXd state_min_inquality(13 * (HORIZON_LENGTH + 1));
  Eigen::VectorXd input_max_inquality(12 * HORIZON_LENGTH);
  Eigen::VectorXd input_min_inquality(12 * HORIZON_LENGTH);

  state_max_inquality = state_max.replicate(HORIZON_LENGTH + 1, 1);
  state_min_inquality = state_min.replicate(HORIZON_LENGTH + 1, 1);
  input_max_inquality = input_max.replicate(HORIZON_LENGTH, 1);
  input_min_inquality = input_min.replicate(HORIZON_LENGTH, 1);

  lower_inequality << state_min_inquality, input_min_inquality;
  upper_inequality << state_max_inquality, input_max_inquality;

  lowerBound << lower_equality, lower_inequality;
  upperBound << upper_equality, upper_inequality;
}

int Mpc_planner_OSQPEigen::run() {
  // TODO：设置求解器 并 solve

  // cast the MPC problem as QP problem
  castMPCToQPHessian();
  castMPCToQPGradient();
  castMPCToQPConstraintMatrix();
  castMPCToQPConstraintVectors();
  // instantiate the solver
  OsqpEigen::Solver solver;

  // settings
  // solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(12 * (HORIZON_LENGTH + 1) + 4 * HORIZON_LENGTH);
  solver.data()->setNumberOfConstraints(2 * 12 * (HORIZON_LENGTH + 1) + 4 * HORIZON_LENGTH);
  if (!solver.data()->setHessianMatrix(hessian)) return 1;
  if (!solver.data()->setGradient(gradient)) return 1;
  if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
  if (!solver.data()->setLowerBound(lowerBound)) return 1;
  if (!solver.data()->setUpperBound(upperBound)) return 1;

  // instantiate the solver
  if (!solver.initSolver()) return 1;

  // controller input and QPSolution vector
  Eigen::VectorXd ctr(12);
  Eigen::VectorXd QPSolution;

  // number of iteration steps
  int numberOfSteps = 50;

  for (int i = 0; i < numberOfSteps; i++) {
    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    ctr = QPSolution.block(12 * (HORIZON_LENGTH + 1), 0, 12, 1);

    // save data into file
    auto x0Data = x_0_.data();

    // propagate the model
    x_0_ = A_ * x_0_ + B_ * ctr.cast<float>();

    // update the constraint bound
    // updateConstraintVectors(x_0_, lowerBound, upperBound);
    // if (!solver.updateBounds(lowerBound, upperBound)) return 1;
  }
}

void Mpc_planner_OSQPEigen::get_resulte() {
  // TODO：获取优化结果
}

Eigen::Matrix3f Mpc_planner_OSQPEigen::coordinateRotation(CoordinateAxis axis, double theta) {
  //   static_assert(std::is_floating_point<T>::value, "must use floating point
  //   value");
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

Eigen::Matrix<float, 3, 3> Mpc_planner_OSQPEigen::cross_mat(
    Eigen::Matrix<float, 3, 3> I_inv, Eigen::Matrix<float, 3, 1> r) {
  Eigen::Matrix<float, 3, 3> cm;
  cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;
  return I_inv * cm;
}