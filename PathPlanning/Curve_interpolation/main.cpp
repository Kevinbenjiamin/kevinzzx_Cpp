


#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;


int main(){

    //换道场景路段与车辆相关参数定义
    double d = 3.5;//道路标准宽度
    double len_line = 30; //直线段长度
    double W=1.75; //车宽
    double L=4.7; //车长

    //车辆换道初始状态与终点期望状态
    double t0=0,t1=3;
    //分别表示小车的x,y; vx,vy; ax,ay
    VectorXd state_t0(6),state_t1(6);
    state_t0 << 0, -d/2, 5, 0, 0, 0;
    state_t1 << 20, d/2, 5, 0, 0, 0;

    //把起末两点的横纵向方程统一用矩阵表达
    VectorXd X(6),Y(6);
    X<<state_t0[0],state_t0[2],state_t0[4],state_t1[0],state_t1[2],state_t1[4];
    Y<<state_t0[1],state_t0[3],state_t0[5],state_t1[1],state_t1[3],state_t1[5];

    MatrixXd T(6,6);
    T<<pow(t0, 5),pow(t0, 4),pow(t0, 3),pow(t0, 2),t0,1,
        5*pow(t0,4),4*pow(t0,3),3*pow(t0,2),2*t0,1,0,
        20*pow(t0,3),12*pow(t0,3),6*t0,1,0,0,
        pow(t1, 5),pow(t1, 4),pow(t1, 3),pow(t1, 2),t1,1,
        5*pow(t1,4),4*pow(t1,3),3*pow(t1,2),2*t1,1,0,
        20*pow(t1,3),12*pow(t1,3),6*t1,1,0,0;

    //计算A和B的两个系数矩阵
    MatrixXd A = T.inverse()*X;
    MatrixXd B = T.inverse()*Y;
    vector<double> x_,y_,v_lon,v_lat;//用于保存数据，横纵向位置、横纵向速度

    vector<double>time;
    int cnt = 0;
    for(double t=t0;t<t1+0.05;t+=0.05) {
        cnt++;
        time.push_back(t);

    }

    MatrixXd temp1(1,6),temp2(1,6);
    for(int i=0;i<cnt;i++){
        temp1<<pow(time[i], 5),pow(time[i], 4),pow(time[i], 3),pow(time[i], 2),time[i],1;
        x_.push_back((temp1*A)(0,0));
        y_.push_back((temp1*B)(0,0));

        temp2<<5*pow(time[i],4),4*pow(time[i],3),3*pow(time[i],2),2*time[i],1,0;
        v_lon.push_back((temp2*A)(0,0));
        v_lat.push_back((temp2*B)(0,0));
    }

    Mat img(500,500,CV_8UC3,cv::Scalar(255,255,255));

    for(int i=1;i<x_.size();i++){
        line(img,Point(x_[i-1]*10,250-y_[i-1]*10),Point(x_[i]*10,250-y_[i]),Scalar(255,0,0),2);
    }
    imshow("curve_interpolation",img);
    waitKey(0);
    return 0;
}

