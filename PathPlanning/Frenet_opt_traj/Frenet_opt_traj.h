
#ifndef FRENET_OPT_TRAJ_H
#define FRENET_OPT_TRAJ_H


#include<iostream>
#include<limits>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sys/time.h>
#include"../utils/cubic_spline.h"
#include"../utils/frenet_path.h"
#include"../utils/quintic_polynomial.h"
#include"../utils/quartic_polynomial.h"

#define SIM_LOOP 500
#define MAX_SPEED  50.0 / 3.6  // maximum speed [m/s]
#define MAX_ACCEL  2.0  // maximum acceleration [m/ss]
#define MAX_CURVATURE  1.0  // maximum curvature [1/m]
#define MAX_ROAD_WIDTH  7.0  // maximum road width [m]
#define D_ROAD_W  1.0  // road width sampling length [m]
#define DT  0.2  // time tick [s]
#define MAXT  5.0  // max prediction time [m]
#define MINT  4.0  // min prediction time [m]
#define TARGET_SPEED  30.0 / 3.6  // target speed [m/s]
#define D_T_S  5.0 / 3.6  // target speed sampling length [m/s]
#define N_S_SAMPLE  1  // sampling number of target speed
#define ROBOT_RADIUS  1.5  // robot radius [m]

#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0

using namespace kevinzzxcpp;

float sum_of_power(std::vector<float> value_list);

Vec_Path calc_frenet_paths(
    float c_speed, float c_d, float c_d_d, float c_d_dd, float s0);


void calc_global_paths(Vec_Path & path_list, Spline2D csp);


bool check_collision(FrenetPath path, const Vec_Poi ob);

Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob);

FrenetPath frenet_optimal_planning(
    Spline2D csp, float s0, float c_speed,
    float c_d, float c_d_d, float c_d_dd, Vec_Poi ob);

cv::Point2i cv_offset(
        float x, float y, int image_width, int image_height);


#endif // FRENET_OPT_TRAJ_H