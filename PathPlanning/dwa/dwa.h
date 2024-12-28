//reference of cpprobotics

#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>


#define PI 3.141592653

using Traj = std::vector<std::array<float, 5>>;
using Obstacle = std::vector<std::array<float, 2>>;
using State = std::array<float, 5>;
using Window = std::array<float, 4>;
using Point = std::array<float, 2>;
using Control = std::array<float, 2>;


class Config{
public:
  float max_speed = 1.0;
  float min_speed = -0.5;
  float max_yawrate = 40.0 * PI / 180.0;
  float max_accel = 0.2;
  float robot_radius = 1.0;
  float max_dyawrate = 40.0 * PI / 180.0;

  float v_reso = 0.01;
  float yawrate_reso = 0.1 * PI / 180.0;

  float dt = 0.1;
  float predict_time = 3.0;
  float to_goal_cost_gain = 1.0;
  float speed_cost_gain = 1.0;
};

State motion(State x, Control u, float dt);

Window calc_dynamic_window(State x, Config config);

Traj calc_trajectory(State x, float v, float y, Config config);

float calc_obstacle_cost(Traj traj, Obstacle ob, Config config);

float calc_to_goal_cost(Traj traj, Point goal, Config config);


Traj calc_final_input(
  State x, Control& u,
  Window dw, Config config, Point goal,
  std::vector<std::array<float, 2>>ob);

Traj dwa_control(State x, Control & u, Config config,
  Point goal, Obstacle ob);

cv::Point2i cv_offset(
    float x, float y, int image_width, int image_height);


