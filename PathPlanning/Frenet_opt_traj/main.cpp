

#include "Frenet_opt_traj.h"


int main(){

    Vec_f wx({0.0, 10.0, 20.5, 35.0, 70.5});
  Vec_f wy({0.0, -6.0, 5.0, 6.5, 0.0});
  std::vector<Poi_f> obstcles{
    {{20.0, 10.0}},
    {{30.0, 6.0}},
    {{30.0, 8.0}},
    {{35.0, 8.0}},
    {{50.0, 3.0}}
  };

  Spline2D csp_obj(wx, wy);
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;

  for(float i=0; i<csp_obj.s.back(); i+=0.1){
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }

  float c_speed = 10.0/3.6;
  float c_d = 2.0;
  float c_d_d = 0.0;
  float c_d_dd = 0.0;
  float s0 = 0.0;

  float area = 20.0;

  cv::namedWindow("frenet", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<SIM_LOOP; i++){
    FrenetPath final_path = frenet_optimal_planning(
      csp_obj, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles);
    s0 = final_path.s[1];
    c_d = final_path.d[1];
    c_d_d = final_path.d_d[1];
    c_d_dd = final_path.d_dd[1];
    c_speed = final_path.s_d[1];

    if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1]-r_y.back()), 2) <= 1.0){
        break;
    }

    // visualization
    cv::Mat bg(2000, 8000, CV_8UC3, cv::Scalar(255, 255, 255));
    for(unsigned int i=1; i<r_x.size(); i++){
      cv::line(
        bg,
        cv_offset(r_x[i-1], r_y[i-1], bg.cols, bg.rows),
        cv_offset(r_x[i], r_y[i], bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        10);
    }
    for(unsigned int i=0; i<final_path.x.size(); i++){
      cv::circle(
        bg,
        cv_offset(final_path.x[i], final_path.y[i], bg.cols, bg.rows),
        40, cv::Scalar(255, 0, 0), -1);
    }

    cv::circle(
      bg,
      cv_offset(final_path.x.front(), final_path.y.front(), bg.cols, bg.rows),
      50, cv::Scalar(0, 255, 0), -1);

    for(unsigned int i=0; i<obstcles.size(); i++){
      cv::circle(
        bg,
        cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows),
        40, cv::Scalar(0, 0, 255), 5);
    }

    cv::putText(
      bg,
      "Speed: " + std::to_string(c_speed*3.6).substr(0, 4) + "km/h",
      cv::Point2i((int)bg.cols*0.5, (int)bg.rows*0.1),
      cv::FONT_HERSHEY_SIMPLEX,
      5,
      cv::Scalar(0, 0, 0),
      10);


    cv::imshow("frenet", bg);
    cv::waitKey(5);

    // save image in build/bin/pngs
    // struct timeval tp;
    // gettimeofday(&tp, NULL);
    // long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    // std::string int_count = std::to_string(ms);
    // cv::imwrite("./pngs/"+int_count+".png", bg);
  }
  return 0;
}
