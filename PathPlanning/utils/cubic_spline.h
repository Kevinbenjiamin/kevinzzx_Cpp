#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H


#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<Eigen/Eigen>
#include<stdexcept>

#include "kevinzzxcpp_types.h"

namespace kevinzzxcpp {

    //计算输入向量的差分，返回一个新的向量，其中每个元素是输入向量中相邻元素的差值
    Vec_f vec_diff(Vec_f input);

      //计算输入向量的累加和，返回一个新的向量，其中每个元素是输入向量中从开始到当前元素的和
      Vec_f cum_sum(Vec_f input);

      //类用于一维样条插值,通过给定的数据点 (x, y) 构造一个三次样条函数，并提供了计算样条函数值、一阶导数和二阶导数的功能。
      class Spline{
      public:
        Vec_f x;
        Vec_f y;
        int nx;
        Vec_f h;
        Vec_f a;
        Vec_f b;
        Vec_f c;
        //Eigen::VectorXf c;
        Vec_f d;
        
        Spline(){};
      

        //通过给定的数据点 (x_, y_) 构造样条函数。计算样条函数的系数 a, b, c, d。
        Spline(Vec_f x_, Vec_f y_):x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_){
          Eigen::MatrixXf A = calc_A();
          Eigen::VectorXf B = calc_B();
          Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
          float * c_pointer = c_eigen.data();
          //Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
          c.assign(c_pointer, c_pointer+c_eigen.rows());
      
          for(int i=0; i<nx-1; i++){
            d.push_back((c[i+1]-c[i])/(3.0*h[i]));
            b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
          }
        };

       //计算样条函数在 t 处的值
        float calc(float t);

       //计算样条函数在 t 处的一阶导数值
        float calc_d(float t);
      
        //计算样条函数在 t 处的二阶导数值
        float calc_dd(float t);
      
      private:
      //计算用于求解样条系数的矩阵 A
        Eigen::MatrixXf calc_A();
        //计算用于求解样条系数的向量 B
        Eigen::VectorXf calc_B();
       
        //二分查找，用于确定 t 所在的区间
        int bisect(float t, int start, int end);
      };

      //类用于二维样条插值。通过给定的二维数据点 (x, y) 构造两个一维样条函数 sx 和 sy，并提供了计算位置、曲率和航向角的功能
      class Spline2D{
      public:
        Spline sx;
        Spline sy;
        Vec_f s;
      
        Spline2D(Vec_f x, Vec_f y){
          s = calc_s(x, y);
          sx = Spline(s, x);
          sy = Spline(s, y);
        };
      
        //计算参数 s_t 处的位置 (x, y)
        Poi_f calc_postion(float s_t);
      
        //计算参数 s_t 处的曲率
        float calc_curvature(float s_t);
      
        //计算参数 s_t 处的航向角
        float calc_yaw(float s_t);
      
      
      private:
      //计算参数化的路径长度 s
        Vec_f calc_s(Vec_f x, Vec_f y);
      };
}



#endif // CUBIC_SPLINE_H