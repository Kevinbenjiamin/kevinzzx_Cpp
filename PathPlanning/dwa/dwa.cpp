

#include "dwa.h"



State motion(State x, Control u, float dt){
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
  return x;
};

Window calc_dynamic_window(State x, Config config){

  return {{
    std::max((x[3] - config.max_accel * config.dt), config.min_speed),
    std::min((x[3] + config.max_accel * config.dt), config.max_speed),
    std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
    std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
  }};
};


Traj calc_trajectory(State x, float v, float y, Config config){

  Traj traj;
  traj.push_back(x);
  float time = 0.0;
  while (time <= config.predict_time){
    x = motion(x, std::array<float, 2>{{v, y}}, config.dt);
    traj.push_back(x);
    time += config.dt;
  }
  return traj;
};


float calc_obstacle_cost(Traj traj, Obstacle ob, Config config){
  // calc obstacle cost inf: collistion, 0:free
  int skip_n = 2;
  float minr = std::numeric_limits<float>::max();

  for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
    for (unsigned int i=0; i< ob.size(); i++){
      float ox = ob[i][0];
      float oy = ob[i][1];
      float dx = traj[ii][0] - ox;
      float dy = traj[ii][1] - oy;

      float r = std::sqrt(dx*dx + dy*dy);
      if (r <= config.robot_radius){
          return std::numeric_limits<float>::max();
      }

      if (minr >= r){
          minr = r;
      }
    }
  }

  return 1.0 / minr;
};

float calc_to_goal_cost(Traj traj, Point goal, Config config){

  float goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
  float traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  float dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  float error = dot_product / (goal_magnitude * traj_magnitude);
  float error_angle = std::acos(error);
  float cost = config.to_goal_cost_gain * error_angle;

  return cost;
};

Traj calc_final_input(
  State x, Control& u,
  Window dw, Config config, Point goal,
  std::vector<std::array<float, 2>>ob){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Traj best_traj;

    // evalucate all trajectory with sampled input in dynamic window
    for (float v=dw[0]; v<=dw[1]; v+=config.v_reso){
      for (float y=dw[2]; y<=dw[3]; y+=config.yawrate_reso){

        Traj traj = calc_trajectory(x, v, y, config);

        float to_goal_cost = calc_to_goal_cost(traj, goal, config);
        float speed_cost = config.speed_cost_gain * (config.max_speed - traj.back()[3]);
        float ob_cost = calc_obstacle_cost(traj, ob, config);
        float final_cost = to_goal_cost + speed_cost + ob_cost;

        if (min_cost >= final_cost){
          min_cost = final_cost;
          min_u = Control{{v, y}};
          best_traj = traj;
        }
      }
    }
    u = min_u;
    return best_traj;
};


Traj dwa_control(State x, Control & u, Config config,
  Point goal, Obstacle ob){
    // # Dynamic Window control
    Window dw = calc_dynamic_window(x, config);
    Traj traj = calc_final_input(x, u, dw, config, goal, ob);

    return u, traj;
  }

cv::Point2i cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};