//
// Created by chh3213 on 2022/11/25.
//

#include "LQRcontrol.h"
#include "../utils/MyReferencePath.h"
#include "../utils/kinematicModel.h"
#include "../utils/NormalizeAngle.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include "../../matplotlibcpp.h"
// namespace plt = matplotlibcpp;

int main(){
    double dt=0.1; // 时间间隔，单位：s
    double L=2; // 车辆轴距，单位：m
    double v = 2; // 初始速度
    double x_0=0; // 初始x
    double y_0=-3; //初始y
    double psi_0=0; // 初始航向角
    int N = 100;//迭代范围

    MatrixXd Q(3,3);
    Q<<3,0,0,
            0,3,0,
            0,0,3;
    MatrixXd R(2,2);
    R<<2.0,0.0,
            0.0,2;

    //保存机器人（小车）运动过程中的轨迹
    vector<double>x_,y_;
    MyReferencePath referencePath;
    KinematicModel ugv(x_0,y_0,psi_0,v,L,dt);
    LQRControl lqr(N);
    vector<double>robot_state;



    for(int i=0;i<500;i++){
        // plt::clf();
        robot_state = ugv.getState();
        vector<double>one_trial = referencePath.calcTrackError(robot_state);
        double k = one_trial[1],ref_yaw = one_trial[2], s0=one_trial[3];

        double ref_delta = atan2(L*k,1);
        vector<MatrixXd>state_space = ugv.stateSpace(ref_delta,ref_yaw);

        double delta = lqr.lqrControl(robot_state, referencePath.refer_path, s0, state_space[0], state_space[1], Q, R);
        delta = delta+ref_delta;

        ugv.updateState(0,delta);//加速度设为0，恒速

        x_.push_back(ugv.x);
        y_.push_back(ugv.y);
        //cout<<ugv.x<<","<<ugv.y<<endl;
        // //画参考轨迹
        // plt::plot(referencePath.refer_x,referencePath.refer_y,"b--");
        // plt::grid(true);
        // plt::ylim(-5,5);
        // //画图
        // plt::plot(x_, y_,"r");
        // plt::pause(0.01);
    }
    // // save figure
    // const char* filename = "./lqr_demo.png";
    // cout << "Saving result to " << filename << std::endl;
    // plt::save(filename);
    // plt::show();
    //创建白色背景的图像
    int width = 1000, height = 600;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255,255,255));
    // 绘制参考轨迹
    for (size_t i = 1; i < referencePath.refer_x.size(); i++) {
        int x1 = static_cast<int>(referencePath.refer_x[i - 1] * 10); // 缩放
        int y1 = height - static_cast<int>((referencePath.refer_y[i - 1] + 3) * 100); // 偏移并翻转
        int x2 = static_cast<int>(referencePath.refer_x[i] * 10);
        int y2 = height - static_cast<int>((referencePath.refer_y[i] + 3) * 100);
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