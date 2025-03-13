cmake_minimum_required(VERSION 3.21)
project(kevinzzx_Cpp)

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

# A_star
add_executable(astar_demo A_Star/main.cpp A_Star/astar.cpp)
target_link_libraries(astar_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# Dijkstra
add_executable(dijkstra_demo Dijkstra/main.cpp Dijkstra/Dijkstra.cpp)
target_link_libraries(dijkstra_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)


# dwa
add_executable(dwa_demo dwa/main.cpp dwa/dwa.cpp)
target_link_libraries(dwa_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# rrt
add_executable(rrt_demo rrt/main.cpp rrt/rrt.cpp)
target_link_libraries(rrt_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# bezier
add_executable(bezier_demo Bezier/main.cpp Bezier/BezierCurve.cpp)
target_link_libraries(bezier_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

# Bspline
add_executable(bspline_demo B-spline/main.cpp B-spline/BSpline.cpp)
target_link_libraries(bspline_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)

#curve_interpolation
add_executable(curve_inter_demo Curve_interpolation/main.cpp)
target_link_libraries(curve_inter_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)


#debins_path
add_executable(debins_demo Dubins_path/main.cpp Dubins_path/Debins_path.cpp)
target_link_libraries(debins_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)


#frenet_opt_traj
add_executable(frenet_opt_traj_demo Frenet_opt_traj/main.cpp Frenet_opt_traj/Frenet_opt_traj.cpp)
target_link_libraries(frenet_opt_traj_demo PRIVATE ${OpenCV_LIBS} kevinzzx_Cpp)