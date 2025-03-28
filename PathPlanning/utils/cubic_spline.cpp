
#include "cubic_spline.h"

namespace kevinzzxcpp {


    //计算输入向量的差分，返回一个新的向量，其中每个元素是输入向量中相邻元素的差值
    Vec_f vec_diff(Vec_f input){
        Vec_f output;
        for(unsigned int i=1; i<input.size(); i++){
          output.push_back(input[i] - input[i-1]);
        }
        return output;
      };

      //计算输入向量的累加和，返回一个新的向量，其中每个元素是输入向量中从开始到当前元素的和
      Vec_f cum_sum(Vec_f input){
        Vec_f output;
        float temp = 0;
        for(unsigned int i=0; i<input.size(); i++){
          temp += input[i];
          output.push_back(temp);
        }
        return output;
      };



       //计算样条函数在 t 处的值
        float Spline::calc(float t){
          if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
          }
          int seg_id = bisect(t, 0, nx);
          float dx = t - x[seg_id];
          return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
        };

       //计算样条函数在 t 处的一阶导数值
        float Spline::calc_d(float t){
          if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
          }
          int seg_id = bisect(t, 0, nx-1);
          float dx = t - x[seg_id];
          return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
        }
      
        //计算样条函数在 t 处的二阶导数值
        float Spline::calc_dd(float t){
          if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
          }
          int seg_id = bisect(t, 0, nx);
          float dx = t - x[seg_id];
          return 2 * c[seg_id] + 6 * d[seg_id] * dx;
        }
      
      //计算用于求解样条系数的矩阵 A
        Eigen::MatrixXf Spline::calc_A(){
          Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
          A(0, 0) = 1;
          for(int i=0; i<nx-1; i++){
            if (i != nx-2){
              A(i+1, i+1) = 2 * (h[i] + h[i+1]);
            }
            A(i+1, i) = h[i];
            A(i, i+1) = h[i];
          }
          A(0, 1) = 0.0;
          A(nx-1, nx-2) = 0.0;
          A(nx-1, nx-1) = 1.0;
          return A;
        };
        //计算用于求解样条系数的向量 B
        Eigen::VectorXf Spline::calc_B(){
          Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
          for(int i=0; i<nx-2; i++){
            B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
          }
          return B;
        };
       
        //二分查找，用于确定 t 所在的区间
        int Spline::bisect(float t, int start, int end){
          int mid = (start+end)/2;
          if (t==x[mid] || end-start<=1){
            return mid;
          }else if (t>x[mid]){
            return bisect(t, mid, end);
          }else{
            return bisect(t, start, mid);
          }
        }


      //类用于二维样条插值。通过给定的二维数据点 (x, y) 构造两个一维样条函数 sx 和 sy，并提供了计算位置、曲率和航向角的功能

        //计算参数 s_t 处的位置 (x, y)
        Poi_f Spline2D::calc_postion(float s_t){
          float x = sx.calc(s_t);
          float y = sy.calc(s_t);
          return {{x, y}};
        };
      
        //计算参数 s_t 处的曲率
        float Spline2D::calc_curvature(float s_t){
          float dx = sx.calc_d(s_t);
          float ddx = sx.calc_dd(s_t);
          float dy = sy.calc_d(s_t);
          float ddy = sy.calc_dd(s_t);
          return (ddy * dx - ddx * dy)/(dx * dx + dy * dy);
        };
      
        //计算参数 s_t 处的航向角
        float Spline2D::calc_yaw(float s_t){
          float dx = sx.calc_d(s_t);
          float dy = sy.calc_d(s_t);
          return std::atan2(dy, dx);
        };
      
      

      //计算参数化的路径长度 s
        Vec_f Spline2D::calc_s(Vec_f x, Vec_f y){
          Vec_f ds;
          Vec_f out_s{0};
          Vec_f dx = vec_diff(x);
          Vec_f dy = vec_diff(y);
      
          for(unsigned int i=0; i<dx.size(); i++){
            ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
          }
      
          Vec_f cum_ds = cum_sum(ds);
          out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
          return out_s;
        };

    }