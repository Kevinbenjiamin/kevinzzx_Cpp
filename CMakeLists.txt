cmake_minimum_required(VERSION 3.5.0)
project(kevinzzx_Cpp VERSION 0.1.0 LANGUAGES C CXX)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include(GNUInstallDirs)
set(PACKAGE_NAME kevinzzx_Cpp)
#Library target
add_library(kevinzzx_Cpp INTERFACE)

# find_package(PythonLibs REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})
# target_link_libraries(kevinzzx_Cpp INTERFACE ${PYTHON_LIBRARIES})

# find_package(Python3 COMPONENTS NumPy)
# if(Python3_NumPy_FOUND)
#     target_link_libraries(kevinzzx_Cpp INTERFACE
#             Python3::NumPy
#             )
# else()
#     target_compile_definitions(kevinzzx_Cpp INTERFACE WITHOUT_NUMPY)
# endif()
# include_directories(/usr/local/lib/python3.8/dist-packages/numpy/core/include)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE kevinzzx_Cpp)

add_subdirectory(pathtracking bin)
add_subdirectory(PathPlanning bin_planning)