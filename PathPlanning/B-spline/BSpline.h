#ifndef BSPLINE_H
#define BSPLINE_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>

double baseFunction(int i, int k, double u, std::vector<double>node_vector);

std::vector<double> u_quasi_uniform(int n,int k);

std::vector<double> u_piecewise_B_Spline(int n,int k);



#endif // BSPLINE_H