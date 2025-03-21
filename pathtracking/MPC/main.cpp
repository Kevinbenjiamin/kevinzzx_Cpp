
#include "MPC.h"



int main(){

    Vec_f wx({0.0, 60.0, 125.0,  50.0,   75.0,  35.0,  -10.0});
  // Vec_f wy({0.0,  4.0,  -4.0,  4.0,  -4.0,   4.0,  0.0});
  Vec_f wy({0.0,  0.0,  50.0,  65.0,   30.0,  50.0,  -20.0});

  Spline2D csp_obj(wx, wy);
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;
  for(float i=0; i<csp_obj.s.back(); i+=1.0){
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }

  float target_speed = 10.0 / 3.6;
  Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);

  mpc_simulation(r_x, r_y, ryaw, rcurvature, speed_profile, {{wx.back(), wy.back()}});
}
