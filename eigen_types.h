#ifndef PX_ARM_CORE_COMMON_MATH_EIGEN_TYPES_H_
#define PX_ARM_CORE_COMMON_MATH_EIGEN_TYPES_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

template <typename T, int size>
using VecX = typename Eigen::Matrix<T, size, 1>;

template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

template <typename T>
using Mat42 = Eigen::Matrix<T, 4, 2>;

template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

template <typename T>
using Mat43 = Eigen::Matrix<T, 4, 3>;

template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

template <typename T>
using Mat62 = Eigen::Matrix<T, 6, 2>;

template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 24x12 Matrix
template <typename T>
using Mat2412 = Eigen::Matrix<T, 24, 12>;

template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

template <typename T, int size1, int size2>
using MatX = typename Eigen::Matrix<T, size1, size2>;

template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

using RotMat = Eigen::Matrix<double, 3, 3>;
using JacobiMat = Eigen::Matrix<double, 6, 6>;
using TranslationVec = Eigen::Matrix<double, 3, 1>;
using JointStatus = Eigen::Matrix<double, 6, 1>;

using Index2i = Eigen::Array2i;
using Size2i = Eigen::Array2i;
using Length2d = Eigen::Array2d;

template <typename T>
using vector_Vec2 = std::vector<Vec2<T>, Eigen::aligned_allocator<Vec2<T>>>;

template <typename T>
using vector_Vec3 = std::vector<Vec3<T>, Eigen::aligned_allocator<Vec3<T>>>;

#endif  // PX_ARM_CORE_COMMON_MATH_EIGEN_TYPES_H_
