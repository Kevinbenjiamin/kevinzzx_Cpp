

#include"rrt.h"

using namespace std;


int main(){
	std::vector<vector<float> > obstacle_list{
		{5, 5, 1},
		{3, 6, 2},
		{3, 8, 2},
		{3, 10, 2},
		{7, 5, 2},
		{9, 5, 2}};

	Node* start = new Node(0.0, 0.0);
	Node* goal = new Node(6.0, 9.0);

	std::vector<float> rand_area{-2, 15};

	RRT rrt(start, goal, obstacle_list, rand_area, 0.5, 1.0, 5, 500);


	std::vector<Node*> path = rrt.planning();

}

