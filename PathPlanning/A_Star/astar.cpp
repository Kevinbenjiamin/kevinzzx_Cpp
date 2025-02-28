#include "astar.h"


/**
函数功能：计算最后的搜索最短路径
输入：目标节点（goal）,网格范围（reso）,图像img
输出：向量pair rx，ry
*/
std::vector<std::vector<float> > calc_final_path(Node * goal, float reso, cv::Mat& img, float img_reso){
  std::vector<float> rx;
  std::vector<float> ry;
  Node* node = goal;
  while (node->p_node != NULL){
    node = node->p_node;
    rx.push_back(node->x * reso);
    ry.push_back(node->y * reso);
    cv::rectangle(img,
        cv::Point(node->x*img_reso+1, node->y*img_reso+1),
        cv::Point((node->x+1)*img_reso, (node->y+1)*img_reso),
        cv::Scalar(255, 0, 0), -1);
  }
  return {rx, ry};
}

/**
函数功能：计算网格中的障碍物
*/
std::vector<std::vector<int> > calc_obstacle_map(
    std::vector<int> ox, std::vector<int> oy,
    const int min_ox, const int max_ox,
    const int min_oy, const int max_oy,
    float reso, float vr,
    cv::Mat& img, int img_reso){

  int xwidth = max_ox-min_ox;
  int ywidth = max_oy-min_oy;

  std::vector<std::vector<int> > obmap(ywidth, vector<int>(xwidth, 0));

  for(int i=0; i<xwidth; i++){
    int x = i + min_ox;
    for(int j=0; j<ywidth; j++){
      int y = j + min_oy;
      for(int k=0; k<ox.size(); k++){
        float d = std::sqrt(std::pow((ox[k]-x), 2)+std::pow((oy[k]-y), 2));
        if (d <= vr/reso){
          obmap[i][j] = 1;
          cv::rectangle(img,
                        cv::Point(i*img_reso+1, j*img_reso+1),
                        cv::Point((i+1)*img_reso, (j+1)*img_reso),
                        cv::Scalar(0, 0, 0), -1);
          break;
        }
      }
    }
  }
  return obmap;
}

/**
节点验证函数,判断是否遇到障碍物
*/
bool verify_node(Node* node,
                 const vector<vector<int>>& obmap,
                 int min_ox, int max_ox,
                 int min_oy, int max_oy){
  if (node->x < min_ox || node->y < min_oy || node->x >= max_ox || node->y >= max_oy){
    return false;
  }

  if (obmap[node->x-min_ox][node->y-min_oy]) return false;

  return true;
}

/**
函数功能：计算当前节点与目标节点的启发代价，这里采用的是曼哈顿距离
输入：目标节点（n1），当前节点（n2），权重系数（w）
输出：启发代价g
*/
float calc_heristic(Node* n1, Node* n2, float w=1.0){
  return w * std::sqrt(std::pow(n1->x-n2->x, 2)+std::pow(n1->y-n2->y, 2));
}


/**
函数功能：给出八个方向的节点选择
无输入输出
*/
std::vector<Node> get_motion_model(){
  return {Node(1,   0,  1),
          Node(0,   1,  1),
          Node(-1,   0,  1),
          Node(0,   -1,  1),
          Node(-1,   -1,  std::sqrt(2)),
          Node(-1,   1,  std::sqrt(2)),
          Node(1,   -1,  std::sqrt(2)),
          Node(1,    1,  std::sqrt(2))};
}

/**
astar算法主要运行函数
*/
void a_star_planning(float sx, float sy,
                     float gx, float gy,
                     vector<float> ox_, vector<float> oy_,
                     float reso, float rr){ 

  Node* nstart = new Node((int)std::round(sx/reso), (int)std::round(sy/reso), 0.0);
  Node* ngoal = new Node((int)std::round(gx/reso), (int)std::round(gy/reso), 0.0);


  vector<int> ox;
  vector<int> oy;

  int min_ox = std::numeric_limits<int>::max();
  int max_ox = std::numeric_limits<int>::min();
  int min_oy = std::numeric_limits<int>::max();
  int max_oy = std::numeric_limits<int>::min();


  for(float iox:ox_){
      int map_x = (int)std::round(iox*1.0/reso);
      ox.push_back(map_x);
      min_ox = std::min(map_x, min_ox);
      max_ox = std::max(map_x, max_ox);
  }

  for(float ioy:oy_){
      int map_y = (int)std::round(ioy*1.0/reso);
      oy.push_back(map_y);
      min_oy = std::min(map_y, min_oy);
      max_oy = std::max(map_y, max_oy);
  }

  int xwidth = max_ox-min_ox;
  int ywidth = max_oy-min_oy;

  //visualization
  cv::namedWindow("astar", cv::WINDOW_NORMAL);
  int count = 0;
  int img_reso = 5;
  cv::Mat bg(img_reso*xwidth,
             img_reso*ywidth,
             CV_8UC3,
             cv::Scalar(255,255,255));

    cv::rectangle(bg,
                  cv::Point(nstart->x*img_reso+1, nstart->y*img_reso+1),
                  cv::Point((nstart->x+1)*img_reso, (nstart->y+1)*img_reso),
                  cv::Scalar(255, 0, 0), -1);
    cv::rectangle(bg,
                  cv::Point(ngoal->x*img_reso+1, ngoal->y*img_reso+1),
                  cv::Point((ngoal->x+1)*img_reso, (ngoal->y+1)*img_reso),
                  cv::Scalar(0, 0, 255), -1);

  std::vector<std::vector<int> > visit_map(xwidth, vector<int>(ywidth, 0));
  
  std::vector<std::vector<float> > path_cost(xwidth, vector<float>(ywidth, std::numeric_limits<float>::max()));

  path_cost[nstart->x][nstart->y] = 0;

  std::vector<std::vector<int> > obmap = calc_obstacle_map(
                                                  ox, oy,
                                                  min_ox, max_ox,
                                                  min_oy, max_oy,
                                                  reso, rr,
                                                  bg, img_reso);

  // NOTE: d_ary_heap should be a better choice here
  auto cmp = [](const Node* left, const Node* right){return left->sum_cost > right->sum_cost;};
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);

  pq.push(nstart);
  std::vector<Node> motion = get_motion_model();

  while (true){
    Node * node = pq.top();

    if (visit_map[node->x-min_ox][node->y-min_oy] == 1){
      pq.pop();
      delete node;
      continue;
    }else{
      pq.pop();
      visit_map[node->x-min_ox][node->y-min_oy] = 1;
    }

    //结束条件
    if (node->x == ngoal->x && node->y==ngoal->y){
      ngoal->sum_cost = node->sum_cost;
      ngoal->p_node = node;
      break; 
    }
    //遍历8个方向的节点
    for(int i=0; i<motion.size(); i++){
      Node * new_node = new Node(
        node->x + motion[i].x,
        node->y + motion[i].y,
        path_cost[node->x][node->y] + motion[i].sum_cost + calc_heristic(ngoal, node),
        node);
        //注意这里定义新节点时设置它的父节点为node了

      if (!verify_node(new_node, obmap, min_ox, max_ox, min_oy, max_oy)){
        delete new_node;
        continue;
      }

      if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
        delete new_node;
        continue;
      }

      cv::rectangle(bg,
                    cv::Point(new_node->x*img_reso+1, new_node->y*img_reso+1),
                    cv::Point((new_node->x+1)*img_reso, (new_node->y+1)*img_reso),
                    cv::Scalar(0, 255, 0));

      // std::string int_count = std::to_string(count);
      // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
      count++;
      cv::imshow("astar", bg);
      cv::waitKey(5);

      if (path_cost[node->x][node->y]+motion[i].sum_cost < path_cost[new_node->x][new_node->y]){
        path_cost[new_node->x][new_node->y]=path_cost[node->x][node->y]+motion[i].sum_cost; 
        pq.push(new_node);
      }
    }
  }

  calc_final_path(ngoal, reso, bg, img_reso);
  delete ngoal;
  delete nstart;

  // std::string int_count = std::to_string(count);
  // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
  cv::imshow("astar", bg);
  cv::waitKey(5);
};
