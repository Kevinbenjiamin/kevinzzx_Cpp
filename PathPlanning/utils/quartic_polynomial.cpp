#include "quartic_polynomial.h"


namespace kevinzzxcpp {


    float QuarticPolynomial::calc_point(float t){
      return a0 + a1*t + a2*std::pow(t, 2) + a3*std::pow(t, 3) + a4*std::pow(t, 4);
    };
    
    float QuarticPolynomial::calc_first_derivative(float t){
      return a1 + 2*a2*t + 3*a3*std::pow(t, 2) + 4*a4*std::pow(t, 3);
    };
    
    float QuarticPolynomial::calc_second_derivative(float t){
      return 2*a2 + 6*a3*t + 12*a4*std::pow(t, 2);
    };
    
    float QuarticPolynomial::calc_third_derivative(float t){
      return 6*a3 + 24*a4*t;
    };
    
    }
    