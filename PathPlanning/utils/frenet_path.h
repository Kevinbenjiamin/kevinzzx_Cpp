#ifndef FRENET_PATH_H
#define FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>

#include "kevinzzxcpp_types.h"

namespace kevinzzxcpp {

    class FrenetPath{
        public:
            float cd = 0.0;
            float cv = 0.0;
            float cf = 0.0;

            Vec_f t;
            Vec_f d;
            Vec_f d_d;
            Vec_f d_dd;
            Vec_f d_ddd;
            Vec_f s;
            Vec_f s_d;
            Vec_f s_dd;
            Vec_f s_ddd;

            Vec_f x;
            Vec_f y;
            Vec_f yaw;
            Vec_f ds;
            Vec_f c;

            float max_speed;
            float max_accel;
            float max_curvature;
    };
    using Vec_Path=std::vector<FrenetPath>;
}






#endif // FRENET_PATH_H