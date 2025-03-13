

#include "Debins_path.h"


#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>



int main(){
    Vector3d start(1.0,1.0,(double)45/180*PI);
    Vector3d goal(2.0,2.0,(double)-45/180*PI);

    double curvature = -1;
    double step_size = 0.1;

    Dubins dubins;
    Dubins::ResultDubins rd = dubins.dubins_path_planning(start, goal, curvature,step_size);
    
    // 创建空白图像
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // 绘制 Dubins 路径
    for (int i = 1; i < rd.p_x.size(); i++) {
        cv::Point pt1(rd.p_x[i - 1] * 100 + 100,  250- rd.p_y[i - 1] * 100); // 坐标转换
        cv::Point pt2(rd.p_x[i] * 100 + 100,  250- rd.p_y[i] * 100);         // 坐标转换
        cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 2);                   // 绘制蓝色路径
    }

    // 绘制起点（绿色圆）
    cv::Point start_pt(start[0] * 100 + 100, 250 - start[1] * 100);
    cv::circle(img, start_pt, 5, cv::Scalar(0, 255, 0), -1); // 绿色填充圆

    // 绘制终点（红色叉）
    cv::Point goal_pt(goal[0] * 100 + 100, 250 - goal[1] * 100);
    cv::line(img, cv::Point(goal_pt.x - 5, goal_pt.y - 5), cv::Point(goal_pt.x + 5, goal_pt.y + 5), cv::Scalar(0, 0, 255), 2); // 红色叉
    cv::line(img, cv::Point(goal_pt.x - 5, goal_pt.y + 5), cv::Point(goal_pt.x + 5, goal_pt.y - 5), cv::Scalar(0, 0, 255), 2); // 红色叉

    // 添加标题（模式）
    string title = "Mode: " + rd.mode;
    cv::putText(img, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);

    // 显示图像
    cv::imshow("Dubins Curve", img);
    cv::waitKey(0);

    return 0;
}

