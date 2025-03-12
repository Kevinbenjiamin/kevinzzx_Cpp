
#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H


#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>


double factorial(int n);

Eigen::Vector2d bezierCommon(std::vector<Eigen::Vector2d>Ps,double t);

Eigen::Vector2d bezierRecursion(std::vector<Eigen::Vector2d>Ps,double t);




#endif // BEZIERCURVE_H
