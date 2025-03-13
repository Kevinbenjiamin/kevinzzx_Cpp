
#include "Frenet_opt_traj.h"


using namespace kevinzzxcpp;

float sum_of_power(std::vector<float> value_list){
    float sum = 0;
    for(float item:value_list){
      sum += item*item;
    }
    return sum;
  };
  
  Vec_Path calc_frenet_paths(
      float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
    std::vector<FrenetPath> fp_list;
    for(float di=-1*MAX_ROAD_WIDTH; di<MAX_ROAD_WIDTH; di+=D_ROAD_W){
      for(float Ti=MINT; Ti<MAXT; Ti+=DT){
        FrenetPath fp;
        QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
        for(float t=0; t<Ti; t+=DT){
          fp.t.push_back(t);
          fp.d.push_back(lat_qp.calc_point(t));
          fp.d_d.push_back(lat_qp.calc_first_derivative(t));
          fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
          fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
        }
        for(float tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;
            tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
            tv+=D_T_S){
  
          FrenetPath fp_bot = fp;
          QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);
  
          fp_bot.max_speed = std::numeric_limits<float>::min();
          fp_bot.max_accel = std::numeric_limits<float>::min();
          for(float t_:fp.t){
            fp_bot.s.push_back(lon_qp.calc_point(t_));
            fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
            fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
            fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
            if(fp_bot.s_d.back() > fp_bot.max_speed){
              fp_bot.max_speed = fp_bot.s_d.back();
            }
            if(fp_bot.s_dd.back() > fp_bot.max_accel){
              fp_bot.max_accel = fp_bot.s_dd.back();
            }
          }
  
          float Jp = sum_of_power(fp.d_ddd);
          float Js = sum_of_power(fp_bot.s_ddd);
          float ds = (TARGET_SPEED - fp_bot.s_d.back());
  
          fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
          fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
          fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;
  
          fp_list.push_back(fp_bot);
        }
      }
    }
    return fp_list;
  };
  
  void calc_global_paths(Vec_Path & path_list, Spline2D csp){
    for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){
      for(unsigned int i=0; i<path_p->s.size(); i++){
        if (path_p->s[i] >= csp.s.back()){
          break;
        }
        std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
        float iyaw = csp.calc_yaw(path_p->s[i]);
        float di = path_p->d[i];
        float x = poi[0] + di * std::cos(iyaw + M_PI/2.0);
        float y = poi[1] + di * std::sin(iyaw + M_PI/2.0);
        path_p->x.push_back(x);
        path_p->y.push_back(y);
      }
  
      for(int i=0; i<path_p->x.size()-1; i++){
        float dx = path_p->x[i + 1] - path_p->x[i];
        float dy = path_p->y[i + 1] - path_p->y[i];
        path_p->yaw.push_back(std::atan2(dy, dx));
        path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
      }
  
      path_p->yaw.push_back(path_p->yaw.back());
      path_p->ds.push_back(path_p->ds.back());
  
  
      path_p->max_curvature = std::numeric_limits<float>::min();
      for(int i=0; i<path_p->x.size()-1; i++){
        path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
        if(path_p->c.back() > path_p->max_curvature){
          path_p->max_curvature = path_p->c.back();
        }
      }
    }
  };
  
  bool check_collision(FrenetPath path, const Vec_Poi ob){
    for(auto point:ob){
      for(unsigned int i=0; i<path.x.size(); i++){
        float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
        if (dist <= ROBOT_RADIUS * ROBOT_RADIUS){
          return false;
        }
      }
    }
    return true;
  };
  
  Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob){
      Vec_Path output_fp_list;
    for(FrenetPath path:path_list){
      if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && check_collision(path, ob)){
        output_fp_list.push_back(path);
      }
    }
    return output_fp_list;
  };
  
  FrenetPath frenet_optimal_planning(
      Spline2D csp, float s0, float c_speed,
      float c_d, float c_d_d, float c_d_dd, Vec_Poi ob){
    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fp_list, csp);
    Vec_Path save_paths = check_paths(fp_list, ob);
  
    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for(auto path:save_paths){
      if (min_cost >= path.cf){
        min_cost = path.cf;
        final_path = path;
      }
    }
    return final_path;
  };
  
  cv::Point2i cv_offset(
      float x, float y, int image_width=2000, int image_height=2000){
    cv::Point2i output;
    output.x = int(x * 100) + 300;
    output.y = image_height - int(y * 100) - image_height/3;
    return output;
  };



