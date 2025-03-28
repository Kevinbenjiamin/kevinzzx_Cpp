

#include "motion_model.h"

namespace kevinzzxcpp {



    
    std::vector<float> quadratic_interpolation(
        std::array<float, 3> x, std::array<float, 3> y){
      Eigen::Matrix3f A;
      Eigen::Vector3f Y;
      A<< std::pow(x[0], 2), x[0], 1,
          std::pow(x[1], 2), x[1], 1,
          std::pow(x[2], 2), x[2], 1;
      Y<<y[0], y[1], y[2];
    
      Eigen::Vector3f result = A.inverse() * Y;
      float* result_data = result.data();
      std::vector<float> result_array(result_data, result_data+3);
      return result_array;
    }
    
    float interp_refer(std::vector<float> para, float x){
      return para[0] * x * x + para[1] * x + para[2];
    }
    
    

    
    void MotionModel::update(float v_, float delta, float dt){
      state.v = v_;
      state.x = state.x + state.v * std::cos(state.yaw) * dt;
      state.y = state.y + state.v * std::sin(state.yaw) * dt;
      state.yaw = state.yaw + state.v / base_l * std::tan(delta) * dt;
      state.yaw = YAW_P2P(state.yaw);
    };
    
    State MotionModel::update(State state_, float delta, float dt){
      state_.x = state_.x + state_.v * std::cos(state_.yaw) * dt;
      state_.y = state_.y + state_.v * std::sin(state_.yaw) * dt;
      state_.yaw = state_.yaw + state_.v / base_l * std::tan(delta) * dt;
      state_.yaw = YAW_P2P(state_.yaw);
      return state_;
    };
    
    Traj MotionModel::generate_trajectory(Parameter p){
      float n =  p.distance / ds;
      float horizon = p.distance / state.v;
    
      // boost::math::cubic_b_spline<float> spline(
      //   p.steering_sequence.data(), p.steering_sequence.size(),
      //   0, horizon/p.steering_sequence.size());
      std::vector<float> spline = quadratic_interpolation(
        {{0, horizon/2, horizon}},
        p.steering_sequence);
    
      Traj output;
      State state_ = state;
    
      for(float i=0.0; i<horizon; i+=horizon/n){
        float kp = interp_refer(spline, i);
        state_ = update(state_, kp, horizon/n);
        TrajState xyyaw{state_.x, state_.y, state_.yaw};
        output.push_back(xyyaw);
      }
      return output;
    }
    
    TrajState MotionModel::generate_last_state(Parameter p){
      float n = p.distance / ds;
      float horizon = p.distance / state.v;
    
      // boost::math::cubic_b_spline<float> spline(
      //   p.steering_sequence.data(), p.steering_sequence.size(),
      //   0, horizon/p.steering_sequence.size());
      std::vector<float> spline = quadratic_interpolation(
        {{0, horizon/2, horizon}},
        p.steering_sequence);
    
      State state_ = state;
      for(float i=0.0; i<horizon; i+=horizon/n){
          float kp = interp_refer(spline, i);
          state_ = update(state_, kp, horizon/n);
      }
      return TrajState{state_.x, state_.y, state_.yaw};
    }

}


