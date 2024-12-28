

#include "dwa.h"


int main(){
  State x({{0.0, 0.0, PI/8.0, 0.0, 0.0}});
  Point goal({{10.0,10.0}});
  Obstacle ob({
    {{-1, -1}},
    {{0, 2}},
    {{4.0, 2.0}},
    {{5.0, 4.0}},
    {{5.0, 5.0}},
    {{5.0, 6.0}},
    {{5.0, 9.0}},
    {{8.0, 9.0}},
    {{7.0, 9.0}},
    {{12.0, 12.0}}
  });

  Control u({{0.0, 0.0}});
  Config config;
  Traj traj;
  traj.push_back(x);

  bool terminal = false;

  cv::namedWindow("dwa", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<1000 && !terminal; i++){
    Traj ltraj = dwa_control(x, u, config, goal, ob);
    x = motion(x, u, config.dt);
    traj.push_back(x);


    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    cv::circle(bg, cv_offset(goal[0], goal[1], bg.cols, bg.rows),
               30, cv::Scalar(255,0,0), 5);
    for(unsigned int j=0; j<ob.size(); j++){
      cv::circle(bg, cv_offset(ob[j][0], ob[j][1], bg.cols, bg.rows),
                 20, cv::Scalar(0,0,0), -1);
    }
    for(unsigned int j=0; j<ltraj.size(); j++){
      cv::circle(bg, cv_offset(ltraj[j][0], ltraj[j][1], bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);
    }
    cv::circle(bg, cv_offset(x[0], x[1], bg.cols, bg.rows),
               30, cv::Scalar(0,0,255), 5);


    cv::arrowedLine(
      bg,
      cv_offset(x[0], x[1], bg.cols, bg.rows),
      cv_offset(x[0] + std::cos(x[2]), x[1] + std::sin(x[2]), bg.cols, bg.rows),
      cv::Scalar(255,0,255),
      7);

    if (std::sqrt(std::pow((x[0] - goal[0]), 2) + std::pow((x[1] - goal[1]), 2)) <= config.robot_radius){
      terminal = true;
      for(unsigned int j=0; j<traj.size(); j++){
        cv::circle(bg, cv_offset(traj[j][0], traj[j][1], bg.cols, bg.rows),
                    7, cv::Scalar(0,0,255), -1);
      }
    }


    cv::imshow("dwa", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;
  }
}
