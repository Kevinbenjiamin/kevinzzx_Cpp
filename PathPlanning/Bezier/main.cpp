

#include "BezierCurve.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;


int main(){

    vector<Vector2d>Ps{Vector2d (0,0),Vector2d(1,1),Vector2d(2,1),Vector2d(3,0),Vector2d(4,2)};

    vector<double> x_ref,y_ref;
    for(int i=0;i<Ps.size();i++){
        x_ref.push_back(Ps[i][0]);
        y_ref.push_back(Ps[i][1]);
    }
    vector<double> x_,y_;
    
    Mat img(500,500,CV_8UC3,cv::Scalar(255,255,255));
    
    for(int t=0;t<100;t++){
        img.setTo(Scalar(255,255,255));
        Vector2d pos = bezierCommon(Ps, (double)t/100);
        x_.push_back(pos[0]);
        y_.push_back(pos[1]);
        //画图
        for(int i=0;i<x_ref.size();i++){
            circle(img, Point(x_ref[i]*100, 500-y_ref[i]*100), 5, Scalar(0,0,255),-1);
        }

        for(int i=1;i<x_.size();i++){
            line(img,Point(x_[i-1]*100,500-y_[i-1]*100),Point(x_[i]*100,500-y_[i]*100),Scalar(255,0,0),2);
        }
        imshow("Bezier Curve",img);
        waitKey(10);

    }
    waitKey(0);
    return 0;

}