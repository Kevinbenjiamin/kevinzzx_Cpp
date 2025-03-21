#ifndef MPC_H
#define MPC_H



#include<iostream>
#include<iomanip>
#include<limits>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sys/time.h>
#include<Eigen/Eigen>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>
#include"../../PathPlanning/utils/cubic_spline.h"
#include"../utils/motion_model.h"
#include"../../PathPlanning/utils/kevinzzxcpp_types.h"


#define NX 4
#define T 6

#define DT 0.2
#define MAX_STEER 45.0/180*M_PI
#define MAX_DSTEER  30.0/180*M_PI

#define MAX_ITER 3
#define DU_TH 0.1

#define N_IND_SEARCH 10
#define MAX_TIME 5000

#define WB 2.5
#define MAX_SPEED   55.0/3.6
#define MIN_SPEED  -20.0/3.6
#define MAX_ACCEL 1.0


#define LENGTH  4.5
#define WIDTH 2.0
#define BACKTOWHEEL 1.0
#define WHEEL_LEN 0.3
#define WHEEL_WIDTH 0.2
#define TREAD 0.7
#define WB 2.5




extern int x_start;
extern int y_start;
extern int yaw_start;
extern int v_start;

extern int delta_start;
extern int a_start;

using namespace kevinzzxcpp;
using M_XREF=Eigen::Matrix<float, NX, T>;

cv::Point2i cv_offset(float x, float y, int image_width=2000, int image_height=2000);



void update(State& state, float a, float delta);



Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed);


int calc_nearest_index(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, int pind);


void calc_ref_trajectory(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f
    ck, Vec_f sp, float dl, int& target_ind, M_XREF& xref);


void smooth_yaw(Vec_f& cyaw);


Vec_f mpc_solve(State x0, M_XREF traj_ref);


void mpc_simulation(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal);


#endif // MPC_H