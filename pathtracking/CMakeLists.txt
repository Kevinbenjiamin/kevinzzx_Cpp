cmake_minimum_required(VERSION 3.5.0)
project(kevinzzx_Cpp VERSION 0.1.0 LANGUAGES C CXX)

set(PACKAGE_NAME kevinzzx_Cpp)


# include_directories(/usr/include/python3.8)
# include_directories(/usr/local/lib/python3.8/dist-packages/numpy/core/include)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

find_package(OsqpEigen REQUIRED)

#PID
add_executable(pid_demo PID/main.cpp PID/pidcontrol.cpp utils/kinematicModel.cpp)
target_link_libraries(pid_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# lqr控制
add_executable(lqr_demo LQR/main.cpp  LQR/LQRcontrol.cpp utils/MyReferencePath.cpp utils/kinematicModel.cpp)
target_link_libraries(lqr_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# Pure Pursuit
add_executable(purepursuit_demo Pure_Pursuit/main.cpp Pure_Pursuit/PurePursuit.cpp utils/kinematicModel.cpp)
target_link_libraries(purepursuit_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# Stanley
add_executable(stanley_demo Stanley/main.cpp Stanley/Stanley.cpp utils/kinematicModel.cpp)
target_link_libraries(stanley_demo PRIVATE kevinzzx_Cpp)

# 后轮位置反馈控制
add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp  Rear_Wheel_Feedback/RearWheelFeedback.cpp utils/MyReferencePath.cpp utils/kinematicModel.cpp)
target_link_libraries(rear_wheel_feedback_demo PRIVATE kevinzzx_Cpp)

#MPC 控制
add_executable(mpc_demo MPC/main.cpp MPC/MPC.cpp utils/motion_model.cpp ../PathPlanning/utils/cubic_spline.cpp)
target_link_libraries(mpc_demo PRIVATE ${OpenCV_LIBS} ipopt kevinzzx_Cpp)
