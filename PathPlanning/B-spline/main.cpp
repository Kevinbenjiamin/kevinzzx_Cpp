#include "BSpline.h"
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

int main(){

    vector<Vector2d>Ps{Vector2d (9.036145, 51.779661),Vector2d(21.084337, 70.084746),Vector2d(37.607573, 50.254237),Vector2d(51.893287, 69.745763),Vector2d(61.187608,  49.576271)};

    vector<double>x_ref,y_ref;
    for(int i=0;i<Ps.size();i++){
        x_ref.push_back(Ps[i][0]);
        y_ref.push_back(Ps[i][1]);
    }
    vector<double>x_,y_;
    Mat img(500,500,CV_8UC3,cv::Scalar(255,255,255));

    int n =Ps.size()-1; //控制点个数-1
    int k = 3; //k阶、k-1次B样条
    Vector2d p_u(0,0);
    vector<double>bik_u(n+1);

    int flag;
    cout<<"请选择：1. 均匀B样条 2.准均匀B样条  3.分段B样条 0. 退出 "<<endl;
    cin>>flag;
    vector<double>node_vector;
    switch (flag) {
        case 1://均匀B样条
            for(int i=0;i<n+k+1;i++){
                node_vector.push_back((double)i/(n+k+1));
            }

            break;
        case 2:
            node_vector= u_quasi_uniform(n,k);

            break;
        case 3:
            node_vector = u_piecewise_B_Spline(n,k);

            break;
        default:
            return 0;
    }

    if(flag==1){
        for(double u = (double)(k-1)/(n+k+1);u<(double)(n + 2) / (n + k+1 );u+=0.005){
            for(int i=0;i<n+1;i++){
                bik_u[i]= baseFunction(i,k,u,node_vector);
                //cout<<bik_u[i]<<endl;
            }
            for(int i=0;i< Ps.size();i++){
                p_u = p_u + Ps[i]*bik_u[i];
            }
            //cout<<p_u<<endl;
            x_.push_back(p_u[0]);
            y_.push_back(p_u[1]);
            p_u=Vector2d (0,0);

        }
    }else{
        for(double u = 0;u<1;u+=0.005){
            for(int i=0;i<n+1;i++){
                bik_u[i]= baseFunction(i,k,u,node_vector);
            }
            for(int i=0;i< Ps.size();i++){
                p_u = p_u + Ps[i]*bik_u[i];
                //cout<<p_u<<","<<endl;
            }
            x_.push_back(p_u[0]);
            y_.push_back(p_u[1]);
            p_u=Vector2d (0,0);

        }
    }
    for(int i=0;i<x_ref.size();i++){
        circle(img, Point(x_ref[i]*3, 500-y_ref[i]*3), 5, Scalar(0,0,255),-1);
    }

    for(int i=1;i<x_.size();i++){
        line(img,Point(x_[i-1]*3,500-y_[i-1]*3),Point(x_[i]*3,500-y_[i]*3),Scalar(255,0,0),2);
    }
    imshow("BSpline Curve",img);
    waitKey(0);
    return 0;


}