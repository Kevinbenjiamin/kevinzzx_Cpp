

#include "../utils/kinematicModel.h"
#include "pidcontrol.h"
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include "../../matplotlibcpp.h"
// namespace plt = matplotlibcpp;
/**
 * PID控制器实现路径跟踪
 */


#define PI 3.1415926


/**
 * 得到距离参考轨迹最近点的下标
 * @param robot_state 机器人状态（x,y）
 * @param refer_path 参考路径
 * @return 距离参考轨迹最近点的下标
 */
double calTargetIndex(vector<double>robot_state, vector<vector<double>>refer_path){
    vector<double>dists;
    for (vector<double>xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
}

int main(){
    //visualization
    vector<vector<double>>refer_path(1000,vector<double>(2));
    vector<double>refer_x,refer_y; //保存参考数据用于画图
    // 生成参考轨迹
    for(int i=0;i<1000;i++){
        refer_path[i][0]=0.1*i;
        refer_path[i][1]=2*sin(refer_path[i][0]/3.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
//        cout<<refer_path[i][0]<<" ,"<<refer_path[i][1]<<endl;
    }

    // 运动学模型
    KinematicModel ugv(0,-1,0.5,2,2,0.1);

    //PID控制器
    PID_controller PID(2,0.01,30,0.,PI/6,-PI/6);
    //保存机器人（小车）运动过程中的轨迹
    vector<double>x_,y_;
    //机器人状态
    vector<double>robot_state(2);
    //运行500个回合
    for(int i=0;i<500;i++){
        // plt::clf();

        robot_state[0]=ugv.x;
        robot_state[1]=ugv.y;
        //参考博客中的公式
        double min_ind = calTargetIndex(robot_state,refer_path);
        double alpha = atan2(refer_path[min_ind][1]-robot_state[1], refer_path[min_ind][0]-robot_state[0]);
        double l_d = sqrt(pow(refer_path[min_ind][0]-robot_state[0],2)+pow(refer_path[min_ind][1]-robot_state[1],2));
        double theta_e = alpha-ugv.psi;
        double e_y = -l_d*sin(theta_e);
        double delta_f = PID.calOutput(e_y);
        //更新机器人状态
        ugv.updateState(0,delta_f);
        x_.push_back(ugv.x);
        y_.push_back(ugv.y);
        //cout<<ugv.x<<","<<ugv.y<<endl;

        // plt::plot(refer_x,refer_y,"b--");
        // plt::plot(x_, y_,"r");
        // plt::grid(true);
        // plt::ylim(-2.5,2.5);
        // plt::pause(0.01);

    }
    // // save figure
    // plt::show();
    // 创建白色背景的图像
    int width = 1000, height = 600;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255,255,255));
    // 绘制参考轨迹
    for (size_t i = 1; i < refer_x.size(); i++) {
        int x1 = static_cast<int>(refer_x[i - 1] * 10); // 缩放
        int y1 = height - static_cast<int>((refer_y[i - 1] + 3) * 100); // 偏移并翻转
        int x2 = static_cast<int>(refer_x[i] * 10);
        int y2 = height - static_cast<int>((refer_y[i] + 3) * 100);
        //cv::line(image, cv::Point(refer_x[i-1], refer_y[i-1]), cv::Point(refer_x[i], refer_y[i]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    // 绘制控制轨迹
    for (size_t i = 1; i < x_.size(); i++) {
        int x1 = static_cast<int>(x_[i - 1] * 10);
        int y1 = height - static_cast<int>((y_[i - 1] + 3) * 100);
        int x2 = static_cast<int>(x_[i] * 10);
        int y2 = height - static_cast<int>((y_[i] + 3) * 100);
        //cv::line(image, cv::Point(x_[i-1], y_[i-1]), cv::Point(x_[i], y_[i]), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // 显示图像
    cv::imshow("Trajectory", image);
    cv::waitKey(0);

    return 0;
}