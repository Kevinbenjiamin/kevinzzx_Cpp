#include<iostream>
#include<cmath>
#include<limits>
#include<queue>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>


using namespace std;

//定义节点类
class Node{
public:
  int x;
  int y;
  float sum_cost;
  Node* p_node;

  Node(int x_, int y_, float sum_cost_=0, Node* p_node_=NULL):x(x_), y(y_), sum_cost(sum_cost_), p_node(p_node_){};
};


std::vector<std::vector<float> > calc_final_path(Node * goal, float reso, cv::Mat& img, float img_reso);


std::vector<std::vector<int> > calc_obstacle_map(
    std::vector<int> ox, std::vector<int> oy,
    const int min_ox, const int max_ox,
    const int min_oy, const int max_oy,
    float reso, float vr,
    cv::Mat& img, int img_reso);


bool verify_node(Node* node,
                 const vector<vector<int>>& obmap,
                 int min_ox, int max_ox,
                 int min_oy, int max_oy);


float calc_heristic(Node* n1, Node* n2, float w);

std::vector<Node> get_motion_model();

void a_star_planning(float sx, float sy,
                     float gx, float gy,
                     vector<float> ox_, vector<float> oy_,
                     float reso, float rr);

