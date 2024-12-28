

#include<iostream>
#include<limits>
#include<random>
#include<vector>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
//#include "../utils/kevinzzxcpp_types.h"

//namespace kevinzzxcpp{

class Node{
public:
	float x;
	float y;
  std::vector<float> path_x;
  std::vector<float> path_y;
	Node* parent;
  float cost;

	Node(float x_, float y_): x(x_), y(y_), parent(NULL), cost(0){};
};



class RRT{
public:
	RRT(Node*, Node*, std::vector<std::vector<float> >, 
		  std::vector<float>, float, float, int, int);

	std::vector<Node*> planning();
	
	Node* GetNearestNode(const std::vector<float>);
	
	bool CollisionCheck(Node*);

protected:

  Node* steer(Node* , Node*, float extend_length);

	Node* start;
	Node* end;
	const float expand_dis;
	const float path_resolution;
	const int goal_sample_rate;
	const int max_iter;
	const std::vector<std::vector<float> > ob_list;

	std::vector<float> rand_area;
	std::vector<Node*> node_list;

	std::random_device goal_rd;
  std::mt19937 goal_gen;
  std::uniform_int_distribution<int> goal_dis;
	
	std::random_device area_rd;
  std::mt19937 area_gen;
  std::uniform_real_distribution<float> area_dis;

  static std::vector<float> calc_distance_and_angle(Node*, Node*);
};
//}