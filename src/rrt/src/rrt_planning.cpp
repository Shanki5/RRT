#include<iostream>
#include<vector>
#include<geometry_msgs/Point.h>
#include<visualization_msgs/Marker.h> 
#include<ros/ros.h>
#include<random>
#include<fstream>


#define goal_bias 0.5



static std::vector<visualization_msgs::Marker> obsVec;

//Node is the data structure used to encompass the waypoints of the path
struct Node
{
  Node(){}

  Node(geometry_msgs::Point p) : point(p) {}

  int id;
  geometry_msgs::Point point;
  std::vector<Node> children;
  int ParentId;
};

class RRT
{

  //the data members
  private:

  Node init_, goal_;
  float sigma_;
  int x_max_;
  int x_min_;
  int y_max_;
  int y_min_;
  std::vector<Node> nodes_list_;
  std::vector<Node> parent_list_;
  std::map<float, Node> distance_map_;

  public: 


  //the constructor
  RRT(Node init, Node goal, float sigma, int x_max, int x_min, int y_max, int y_min)
  :init_(init), goal_(goal), sigma_(sigma), x_max_(x_max), x_min_(x_min), y_max_(y_max), y_min_(y_min) 
  {
    nodes_list_.reserve(1000);
    nodes_list_.push_back(init);
  }

  // generates a random point
  geometry_msgs::Point getRandomConfig() 
  {
    geometry_msgs::Point point;

    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(x_min_, x_max_);

    point.x = distr(generator);
    point.y = distr(generator);

    return point;
  }

  // returns the nearest node in the tree for the given point 
  Node getNearestNode(geometry_msgs:: Point p)
  {
    if(nodes_list_.size() == 1)
      return nodes_list_[0];
    distance_map_.clear();

    for(auto node: nodes_list_)
    {
      float distance = getEuclideanDistance(p, node.point);
      distance_map_.insert({distance,node});  
    }
    
    return distance_map_.begin()->second;
  }

  // calculates euclidean distance b/w two points
  float getEuclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
  {
    return std::sqrt(std::pow((p1.x - p2.x),2) + std::pow((p1.y - p2.y),2));
  }

  // checks if the given point is within the map
  bool isWithinMap(geometry_msgs::Point p)
  {
    return (p.x > x_min_ && p.x < x_max_ && p.y > y_min_ && p.y < y_max_ );
  }

  // checks if the path between two points is obstructed
  bool intersectsObs(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obsVec) 
  {

    float x1 = p1.x;
    float y1 = p1.y;
    float x2 = p2.x;
    float y2 = p2.y;

    for (auto obs: obsVec) 
    {
      float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5;
      float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5;
      float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5;
      float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5;

      //check for the bottom intersection
      bool bottom = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);
      //left intersect
      bool left = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);
      //right intersect
      bool right = lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);
      //top intersect
      bool top = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

      if (bottom || left || right || top) 
      {
        return true;
      }
    }
    return false;
  }

  bool lineIntersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

    // calculate the distance to intersection point
    float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

    // if uA and uB are between 0-1, lines are colliding
    if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) 
    {
      float intersectionX = x1 + (uA * (x2 - x1));
      float intersectionY = y1 + (uA * (y2 - y1));
      return true;
    }
    return false;
  }

  // expands the tree 
  Node expand(Node p1, Node p2, std::vector<visualization_msgs::Marker> obsVec, int frameid) 
  {
    //calculate the slope
    float m, nume, denom;
    if (p1.point.x != p2.point.x) 
    {
      nume = (p2.point.y - p1.point.y);
      denom = (p2.point.x - p1.point.x);
      m = nume / denom;
    }
    float theta = atan(m);
    if (theta < 0) 
    {
      if (denom < 0) 
      {
        theta = theta + M_PI;
      } 
      else 
      {
        theta = theta + 2 * M_PI;
      }
    } 
    else 
    {
      if ((nume < 0) && (denom < 0)) 
      {
        theta = theta + M_PI;
      }
    }
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    //calculate P
    Node p;
    p.point.y = sigma_ * sin_theta + p1.point.y;
    p.point.x = sigma_ * cos_theta + p1.point.x;
    p.point.z = 0;
    p.id = frameid;

    // calculate if the point is within an obstacle && isWithinMap(p.point)
    if (!intersectsObs(p1.point, p.point, obsVec) ) 
    {
        std::vector<Node>::iterator it = parent_list_.begin();
        it = parent_list_.insert(it, p1);

        p.ParentId = p1.id;
        p1.children.push_back(p); //children of init is not in the nodeslist

        nodes_list_.push_back(p);
        return p;
    }
    return p1;
  }

  std::vector<Node> getNodesList() 
  {
    return this->nodes_list_;
  }
};


// Helper functions
void populateRviz(ros::Publisher marker_pub, Node init, Node goal, std::string filename);

Node runRRT(ros::Publisher marker_pub, int frame_count, RRT& rrt, Node goal);

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher, bool);

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub);

void populateObstacles(ros::Publisher marker_pub, std::string filename);

bool moveRobot(ros::Publisher marker_pub, geometry_msgs::Point, Node goal);


int main(int argc, char **argv)
{

  std::string param;
  Node init, goal;
  init.id = -1;
  init.ParentId = -2;
  goal.id = 10000;

  std::cout << "Enter start coordinates (should be less than 20): \n";
  std::cin >> init.point.x >> init.point.y;

  std::cout << "Enter goal coordinates (should be less than 20): \n";
  std::cin >> goal.point.x >> goal.point.y;

  float sigma = 0.5;

  static RRT rrt(init, goal, sigma, 20, 0, 20, 0);

  ros::init(argc, argv, "RRT");
  ros::NodeHandle n("~");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
  ros::Rate loop_rate(20);

  n.getParam("obstacleFile", param);

  std::cout << "Obstacle File: " << param << std::endl;
  std::cin.get();

  static int frame_count = 0;
  static bool success = false;

  while(ros::ok())
  {
    ROS_INFO("Frame: %d", frame_count);
    populateRviz(marker_pub,init,goal, param);

    if(!success)
    {
      Node next_node = runRRT(marker_pub, frame_count,rrt,goal);
      geometry_msgs::Point next_point = next_node.point;

      if ((rrt.getEuclideanDistance(next_point, goal.point) <= 1) && frame_count > 2) 
      {
        addEdge(next_point, goal.point, marker_pub, false);
        next_node.children.push_back(goal);
        goal.ParentId = next_node.id;
        success = true;
      }
  
    }

    if (success) 
    {
      std::vector<Node> pathNodes;
      std::vector<Node> allNodes = rrt.getNodesList();

      pathNodes.push_back(goal);
      int tempParentId = goal.ParentId;
      while (tempParentId != init.ParentId) 
      {
        for (int i = allNodes.size() - 1; i >= 0; i--) 
        {
          Node tempNode = allNodes[i];
          if ((tempNode.id) == tempParentId) 
          {
            pathNodes.push_back(tempNode);
            tempParentId = tempNode.ParentId;
          }
        }
      }

      std::cout << "\n\nPath retrieved!! \n\n";

      Node next;
      Node curr;

      for (int i = pathNodes.size() - 2; i >= 0; i--) 
      {
        curr = pathNodes[i];
        next = pathNodes[i + 1];
        drawFinalPath(curr.point, next.point, marker_pub);
      }
    }

    while (marker_pub.getNumSubscribers() < 1) 
    {
      if (!ros::ok()) 
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }

      //iterate ROS
      ros::spinOnce();
      loop_rate.sleep();
      ++frame_count;
  }

  return 0;
}

Node runRRT(ros::Publisher marker_pub, int frameid, RRT& rrt, Node goal) 
{
  geometry_msgs::Point rand_point = rrt.getRandomConfig();
  geometry_msgs::Point tempP;
  tempP.x = 0;
  tempP.y = 0;
  Node rand_node(tempP);
  Node next_node(tempP);
  Node nearest_node = rrt.getNearestNode(rand_point);

  //decide whether to extend toward the goal or a random point
  double r = rand() / (double) RAND_MAX;
  if (r < goal_bias) {
      next_node = rrt.expand(nearest_node, goal, obsVec, frameid);
  } else {
      rand_node.point = rand_point;
      next_node = rrt.expand(nearest_node, rand_node, obsVec, frameid);
  }

  if ((next_node.point.x != nearest_node.point.x) && (next_node.point.y != nearest_node.point.y)) {
      std::cout << "Rand_config: \n" << rand_point << "nearest_node: \n" << nearest_node.point << "next_node: \n" << (next_node).point << "\n\n";
      addEdge(nearest_node.point, (next_node).point, marker_pub, false);
  }
  return next_node;
}

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
  static visualization_msgs::Marker edge;
  edge.type = visualization_msgs::Marker::LINE_LIST;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.ns = "finalPath";
  edge.id = 4;
  edge.action = visualization_msgs::Marker::ADD;
  edge.pose.orientation.w = 1;

  edge.scale.x = 0.04;

  edge.color.g = edge.color.r = 1;
  edge.color.a = 1.0;

  edge.points.push_back(p1);
  edge.points.push_back(p2);

  marker_pub.publish(edge);
}

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub, bool isFinal) {
    static visualization_msgs::Marker edge, vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.02;
    if (!isFinal) {
        edge.color.r = 1.0;
    } else {
        edge.color.g = edge.color.r = 1;
    }
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void populateRviz(ros::Publisher marker_pub, Node init, Node goal, std::string filename) 
{
  visualization_msgs::Marker v_start, v_end;
  v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
  v_start.header.frame_id = v_end.header.frame_id = "map";
  v_start.header.stamp = v_end.header.stamp = ros::Time::now();
  v_start.ns = v_end.ns = "start/end vertices";
  v_start.id = 0;
  v_end.id = 1;
  v_start.action = v_end.action = visualization_msgs::Marker::ADD;

  v_start.color.a = 1.0f;
  v_start.color.g = 1.0f;
  v_start.scale.x = v_start.scale.y = 0.2;
  v_end.scale.x = v_end.scale.y = 0.2;

  v_end.color.a = 1.0f;
  v_end.color.r = 1.0f;

  geometry_msgs::Point ps, pe;
  ps.x = init.point.x;
  ps.y = init.point.y;
  pe.x = goal.point.x;
  pe.y = goal.point.y;
  v_start.points.push_back(ps);
  v_end.points.push_back(pe);

  //publish edge and vertices
  marker_pub.publish(v_start);
  marker_pub.publish(v_end);

  populateObstacles(marker_pub,filename);

};

void populateObstacles(ros::Publisher marker_pub, std::string filename) 
{
  std::ifstream readfile(filename);
  std::string text;

  int obs_count = 0;

  while (std::getline(readfile,text))
  {
    visualization_msgs::Marker obs_curr;
    obs_curr.type = visualization_msgs::Marker::CUBE;
    obs_curr.header.frame_id = "map";
    obs_curr.header.stamp = ros::Time::now();
    obs_curr.ns = "obstacles";
    obs_curr.lifetime = ros::Duration();
    obs_curr.action = visualization_msgs::Marker::ADD;
    obs_curr.id = obs_count;

    obs_curr.scale.z = 0.25;

    obs_curr.pose.position.z = 0.25;

    obs_curr.pose.orientation.x = 0;
    obs_curr.pose.orientation.y = 0;
    obs_curr.pose.orientation.z = 0;
    obs_curr.pose.orientation.w = 1;

    obs_curr.color.a = 1;
    obs_curr.color.r = obs_curr.color.g = obs_curr.color.b = 6.6f;

    std::stringstream ss(text);
    std::vector<double> v;

    while (ss.good())
    {
      std::string substr;
      getline(ss,substr,',');
      v.push_back(std::stod(substr));  
    }

    obs_curr.scale.x = v[0];
    obs_curr.scale.y = v[1];
    obs_curr.pose.position.x = v[2];
    obs_curr.pose.position.y = v[3];

    marker_pub.publish(obs_curr);

    obsVec.push_back(obs_curr);

    obs_count++;
  }
}


