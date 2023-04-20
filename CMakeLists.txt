cmake_minimum_required(VERSION 2.8)

project(liusong_pro)
include_directories("/usr/include/eigen3")
include_directories("/usr/local/include/osqp")
find_package(OsqpEigen REQUIRED)
# include_directories("/usr/include/boost")
set(CMAKE_BUILD_TYPE "Debug")

# ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")

#aux_source_directory（目录  存放文件列表的变量） 搜索./ls_pro1目录下的所有.cpp文件
# aux_source_directory(./ls_pro1 CPP_LIST1)
# add_executable(
#     out_file1
#     ${CPP_LIST1}
# )
add_definitions( -std=c++14)

INCLUDE_DIRECTORIES("/usr/local/include")
# LINK_LIBRARIES("/usr/local/lib/libqpOASES.a")
add_executable(qp_solver_benchmark main.cpp Mpc_planner_qpOASES.cpp)

target_link_libraries(qp_solver_benchmark
  PUBLIC
    qpOASES
    osqp
    OsqpEigen::OsqpEigen
)
#---------------------------------------------------------------------
# SET(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/models")
# ADD_DEFINITIONS(-DPINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}")
# SET(PINOCCHIO_MODEL_DIR "~/home/lius/Example/models")

# INCLUDE_DIRECTORIES("/opt/openrobots/include")
# LINK_LIBRARIES("/opt/openrobots/lib/libpinocchio.so")

# add_executable(pino1 pino1.cpp)
# add_executable(pino_IK pino_IK.cpp)
# add_executable(test_boost test_boost.cpp)
#-------------------------------------------------------------------

# INCLUDE_DIRECTORIES: 添加头文件(.h)路径;
# LINK_DIRECTORIES: 添加需要链接的库文件目录;
# LINK_LIBRARIES: 添加需要链接的库文件路径;
# link_libraries用在add_executable之前，target_link_libraries用在add_executable之后
# link_libraries用来链接静态库，target_link_libraries用来链接导入库，即按照header file + .lib + .dll方式隐式调用动态库的.lib库
# set(PINO_LIB "${/opt/openrobots/lib}" )
