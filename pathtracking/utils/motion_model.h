
#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H


#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<cfenv>
#include<Eigen/Eigen>


#define YAW_P2P(angle) std::fmod(std::fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI


namespace kevinzzxcpp {

    //存储车辆运动的参数，包括行驶距离和转向序列
    struct Parameter{
        float distance;
        std::array<float, 3> steering_sequence{{0,0,0}};
        Parameter(float distance_, std::array<float, 3> steering_sequence_){
          distance = distance_;
          steering_sequence = steering_sequence_;
        };
      };
      
      //存储车辆的当前状态，包括位置 (x, y)、方向 (yaw) 和速度 (v)
      struct State{
        float x;
        float y;
        float yaw;
        float v;
        State(float x_, float y_, float yaw_, float v_){
          x = x_;
          y = y_;
          yaw = yaw_;
          v = v_;
        };
      };
      
      //存储轨迹点的状态，包括位置 (x, y) 和方向 (yaw)
      struct TrajState{
        float x;
        float y;
        float yaw;
        TrajState(float x_, float y_, float yaw_){
          x = x_;
          y = y_;
          yaw = yaw_;
        };
      };
      using Traj = std::vector<TrajState>;
      using StateList = std::vector<TrajState>;
      using ParameterList = std::vector<Parameter>;
      
      //使用二次插值法计算转向序列的插值参数
      std::vector<float> quadratic_interpolation(
          std::array<float, 3> x, std::array<float, 3> y);
      
    //根据插值参数计算给定时间点的转向值
      float interp_refer(std::vector<float> para, float x);
      
      //模拟车辆的运动，生成轨迹或最终状态
      class MotionModel{
      public:
        const float base_l;
        const float ds;
        State state;
      
        MotionModel(float base_l_, float ds_, State state_):
        base_l(base_l_), ds(ds_), state(state_){};
      
      void update(float v_, float delta, float dt);
      
      State update(State state_, float delta, float dt);
      
      //生成车辆的运动轨迹
      Traj generate_trajectory(Parameter p);
      
      //生成车辆的最终状态
      TrajState generate_last_state(Parameter p);

      };
}




#endif // MOTION_MODEL_H
