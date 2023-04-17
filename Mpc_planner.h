#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "common_data.h"
#include "osqp.h"
#include "qpOASES.hpp"
enum class CoordinateAxis { X, Y, Z };
#define HORIZON_LENGTH 10
#define BIG_NUMBER 100000
class Mpc_planner {
 public:
  Mpc_planner();
  ~Mpc_planner();

  qpOASES::real_t* H_red;
  qpOASES::real_t* g_red;
  qpOASES::real_t* A_red;
  qpOASES::real_t* lbA_red;
  qpOASES::real_t* ubA_red;
  qpOASES::real_t* q_red;
  bool real_allocated;

  void init();
  void update_mpc_parameter(float& dt, int& horizon, float& mu, float& f_max, float& dtMPC);
  void update_dynamic_parameter(const FusionData& fusion_data, const OperatorData& operator_data);
  void update_ABmatrix_parameter(Eigen::Vector3f& inertial, Eigen::Vector3f& com, float mass);
  void update_qpOASES_parameter(Eigen::Matrix<float, 12, 1>& weights, float alpha);
  void run();
  void get_result();
  Eigen::Matrix<float, 3, 3> cross_mat(Eigen::Matrix<float, 3, 3> I_inv, Eigen::Matrix<float, 3, 1> r);
  Eigen::Matrix<float, 3, 3> coordinateRotation(CoordinateAxis axis, float theta);
  template <typename Derived>
  void Eigen2real_t(const Eigen::MatrixBase<Derived>& mat_in, qpOASES::real_t* mat_out, int rows, int cols);

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
  Eigen::Matrix<float, 13 * HORIZON_LENGTH, 13> A_qp_;
  Eigen::Matrix<float, 13 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> B_qp_;
  float mass_;
  float dt_;
  float dtMPC_;
  int horizon_;
  float mu_;
  float f_max_;
  bool first_run_;
  float alpha_;

  Eigen::Matrix<float, 12 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> H_matrix_;
  Eigen::Matrix<float, 12 * HORIZON_LENGTH, 1> g_matrix_;
  Eigen::Matrix<float, 20 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> A_matrix_;
  Eigen::DiagonalMatrix<float, 13 * HORIZON_LENGTH> S_;
  Eigen::Matrix<float, 13, 1> weights_;
  Eigen::Matrix<float, 20 * HORIZON_LENGTH, 1> lbA_matrix_;
  Eigen::Matrix<float, 20 * HORIZON_LENGTH, 1> ubA_matrix_;
};

#endif
