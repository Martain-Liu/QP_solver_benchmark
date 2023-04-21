#ifndef MPC_PLANNER_OSQPEIGEN_H
#define MPC_PLANNER_OSQPEIGEN_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "OsqpEigen/OsqpEigen.h"
#include "common_data.h"
enum class CoordinateAxis { X, Y, Z };
#define HORIZON_LENGTH 10
class Mpc_planner_OSQPEigen {
 public:
  Mpc_planner_OSQPEigen();
  ~Mpc_planner_OSQPEigen();

  void init();
  void get_resulte();
  void set_state_parameter(const FusionData& fusion_data, const OperatorData& operator_data);
  void set_solver_parameter(
      Eigen::Matrix<double, 12, 1>& Q,
      Eigen::Matrix<double, 12, 1>& R,
      Eigen::Matrix<double, 12, 1>& statemax,
      Eigen::Matrix<double, 12, 1>& statemin,
      Eigen::Matrix<double, 12, 1>& inputmax,
      Eigen::Matrix<double, 12, 1>& inputmin);
  int run();
  void castMPCToQPHessian();
  void castMPCToQPGradient();
  void castMPCToQPConstraintMatrix();
  void castMPCToQPConstraintVectors();
  Eigen::Matrix3f coordinateRotation(CoordinateAxis axis, double theta);
  Eigen::Matrix<float, 3, 3> cross_mat(Eigen::Matrix<float, 3, 3> I_inv, Eigen::Matrix<float, 3, 1> r);

  int state_optim_nums;
  int input_optim_nums;
  Eigen::Matrix<double, 13, 1> Q_weights;
  Eigen::Matrix<double, 12, 1> R_weights;
  Eigen::Matrix<double, 13 * (HORIZON_LENGTH + 1), 1> Q_full_weights;
  Eigen::Matrix<double, 12 * HORIZON_LENGTH, 1> R_full_weights;
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> linearMatrix;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd upperBound;
  Eigen::VectorXd state_max;
  Eigen::VectorXd state_min;
  Eigen::VectorXd input_max;
  Eigen::VectorXd input_min;

 private:
  Eigen::Matrix<float, 12, 1> traj_init_;
  Eigen::Matrix<float, 13 * HORIZON_LENGTH, 1> traj_all_;
  Eigen::Matrix<float, 13, 1> x_0_;
  Eigen::Vector3f root_euler_;
  Eigen::Vector3f root_position_;
  Eigen::Vector3f root_angular_velocity_in_world_;
  Eigen::Vector3f root_linear_velocity_in_world_;
  Eigen::Matrix3f rotation_matrix_body_to_world_;
  Eigen::Matrix<float, 4, 3> foot_position_in_body_;
  Eigen::Vector3f I_body_;
  Eigen::Matrix3f I_world_;
  Eigen::Matrix3f I_world_inv_;
  Eigen::Matrix3f R_yaw_;
  Eigen::Matrix<float, 13, 13> A_;
  Eigen::Matrix<float, 13, 12> B_;
  Eigen::Vector3f r_com_to_foot_;
  Eigen::Vector3f com_in_body_;
  float mass_;
  float dt_;
  float dtMPC_;
  int horizon_;
  float mu_;
  float f_max_;
  bool first_run_;
  float alpha_;
};

#endif