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