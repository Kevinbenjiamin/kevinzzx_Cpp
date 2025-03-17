
#include "quintic_polynomial.h"

namespace kevinzzxcpp {

    

          float QuinticPolynomial::calc_point(float t){
            return a0 + a1*t + a2*std::pow(t, 2) + a3*std::pow(t, 3) + a4*std::pow(t, 4) + a5*std::pow(t, 5);
          };
        
          float QuinticPolynomial::calc_first_derivative(float t){
            return a1 + 2*a2*t + 3*a3*std::pow(t, 2) + 4*a4*std::pow(t, 3) + a5*std::pow(t, 4);
          };
        
          float QuinticPolynomial::calc_second_derivative(float t){
            return 2*a2 + 6*a3*t + 12*a4*std::pow(t, 2) + 20*a5*std::pow(t, 3);
          };
        
          float QuinticPolynomial::calc_third_derivative(float t){
            return 6*a3 + 24*a4*t + 60*a5*std::pow(t, 2);
          };

}
