#include<iostream>
#include<vector>
#include<geometry_msgs/Point.h>
#include<visualization_msgs/Marker.h> 
#include<ros/ros.h>
#include<random>

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
void populateRviz(ros::Publisher marker_pub, Node init, Node goal);

Node runRRT(ros::Publisher marker_pub, int frame_count, RRT& rrt, Node goal);

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher, bool);

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub);

void populateObstacles(ros::Publisher marker_pub);

bool moveRobot(ros::Publisher marker_pub, geometry_msgs::Point, Node goal);


int main(int argc, char **argv)
{
  Node init, goal;
  init.id = -1;
  init.ParentId = -2;
  goal.id = 10000;

  std::cout << "Enter start coordinates (should be less than 20): ";
  std::cin >> init.point.x >> init.point.y;

  std::cout << "Enter goal coordinates (should be less than 20): ";
  std::cin >> goal.point.x >> goal.point.y;

  float sigma = 0.5;

  static RRT rrt(init, goal, sigma, 20, 0, 20, 0);

  ros::init(argc, argv, "RRT");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
  // ros::Publisher chatter = 
  ros::Rate loop_rate(20);

  static int frame_count = 0;
  static bool success = false;

  while(ros::ok())
  {
    ROS_INFO("Frame: %d", frame_count);
    populateRviz(marker_pub,init,goal);

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

void populateRviz(ros::Publisher marker_pub, Node init, Node goal) 
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

  populateObstacles(marker_pub);

};

void populateObstacles(ros::Publisher marker_pub) 
{
  visualization_msgs::Marker obs1, obs2, obs3, obs4, obs5, obs6;

  obs1.type = obs2.type = obs3.type = obs4.type = obs5.type = obs6.type = visualization_msgs::Marker::CUBE;
  obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = obs5.header.frame_id = obs6.header.frame_id = "map";
  obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = obs5.header.stamp = obs6.header.stamp = ros::Time::now();
  obs1.ns = obs2.ns = obs3.ns = obs4.ns = obs5.ns = obs6.ns = "obstacles";
  obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = obs5.lifetime = obs6.lifetime = ros::Duration();
  obs1.action = obs2.action = obs3.action = obs4.action = obs5.action = obs6.action = visualization_msgs::Marker::ADD;

  obs1.id = 0;
  obs2.id = 1;
  obs3.id = 2;
  obs4.id = 3;
  obs5.id = 4;
  obs6.id = 5;

  obs1.scale.x = obs2.scale.x = 2;
  obs1.scale.y = obs2.scale.y = 12;
  obs3.scale.y = 4;
  obs3.scale.x = 7;
  obs4.scale.x = obs4.scale.y = 2;
  obs5.scale.x = 5;
  obs5.scale.y = 3;
  obs6.scale.x = 3;
  obs6.scale.y = 7;

  obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = obs5.scale.z = obs6.scale.z = 0.25;


  obs1.pose.position.x = 8;
  obs1.pose.position.y = 6;
  obs1.pose.position.z = 0.25;
  obs1.pose.orientation.x = 0.0;
  obs1.pose.orientation.y = 0.0;
  obs1.pose.orientation.z = 0.0;
  obs1.pose.orientation.w = 1;
  obs1.color.a = 1;
  obs1.color.r = obs1.color.g = obs1.color.b = 6.6f;

  obs2.pose.position.x = 14;
  obs2.pose.position.y = 14;
  obs2.pose.position.z = 0.25;
  obs2.pose.orientation.x = 0.0;
  obs2.pose.orientation.y = 0.0;
  obs2.pose.orientation.z = 0.0;
  obs2.pose.orientation.w = 1;
  obs2.color.a = 1;
  obs2.color.r = obs2.color.g = obs2.color.b = 6.6f;

  obs3.pose.position.x = 16.5;
  obs3.pose.position.y = 2;
  obs3.pose.position.z = 0.25;
  obs3.pose.orientation.x = 0.0;
  obs3.pose.orientation.y = 0.0;
  obs3.pose.orientation.z = 0.0;
  obs3.pose.orientation.w = 1;
  obs3.color.a = 1;
  obs3.color.r = obs3.color.g = obs3.color.b = 6.6f;

  obs4.pose.position.x = 16;
  obs4.pose.position.y = 9;
  obs4.pose.position.z = 0.25;
  obs4.pose.orientation.x = 0.0;
  obs4.pose.orientation.y = 0.0;
  obs4.pose.orientation.z = 0.0;
  obs4.pose.orientation.w = 1;
  obs4.color.a = 1;
  obs4.color.r = obs4.color.g = obs4.color.b = 6.6f;

  obs5.pose.position.x = 2.5;
  obs5.pose.position.y = 18.5;
  obs5.pose.position.z = 0.25;
  obs5.pose.orientation.x = 0.0;
  obs5.pose.orientation.y = 0.0;
  obs5.pose.orientation.z = 0.0;
  obs5.pose.orientation.w = 1;
  obs5.color.a = 1;
  obs5.color.r = obs5.color.g = obs5.color.b = 6.6f;

  obs6.pose.position.x = 1.5;
  obs6.pose.position.y = 13.5;
  obs6.pose.position.z = 0.25;
  obs6.pose.orientation.x = 0.0;
  obs6.pose.orientation.y = 0.0;
  obs6.pose.orientation.z = 0.0;
  obs6.pose.orientation.w = 1;
  obs6.color.a = 1;
  obs6.color.r = obs6.color.g = obs6.color.b = 6.6f;

  marker_pub.publish(obs1);
  marker_pub.publish(obs2);
  marker_pub.publish(obs3);
  marker_pub.publish(obs4);
  marker_pub.publish(obs5);
  marker_pub.publish(obs6);

  obsVec.push_back(obs1);
  obsVec.push_back(obs2);
  obsVec.push_back(obs3);
  obsVec.push_back(obs4);
  obsVec.push_back(obs5);
  obsVec.push_back(obs6);
}


