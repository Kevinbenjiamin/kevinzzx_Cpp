

#include "BezierCurve.h"
#include "Eigen/src/Core/Matrix.h"
#include <cmath>
#include <math.h>


/**
阶乘实现

*/
double factorial(int n){
    if (n<=1) {
        return 1;
    }
    return factorial(n-1)*n;
}



/**
贝塞尔公式
*/

Eigen::Vector2d bezierCommon(std::vector<Eigen::Vector2d> Ps, double t){
    if (Ps.size()==1) return Ps[0];
    Eigen::Vector2d p_t(0.,0.);
    int n = Ps.size()-1;
    for(int i=0;i<Ps.size();i++){
        double C_n_i = factorial(n)/(factorial(i)*factorial(n-i));
        p_t += C_n_i*pow((1-t), (n-i))*pow(t,i)*Ps[i];
    }
    return p_t;
}
