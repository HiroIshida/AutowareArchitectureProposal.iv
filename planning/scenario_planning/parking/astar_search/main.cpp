#include "astar_search/astar_search.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <vector>

using namespace std;

/* obtained from std output
start pose
position: 3716.06, 73751.3, 0.0541427
orientation: 8.21603e-05, 5.30234e-05, 0.228649, 0.973509
goal pose
position: 3719.11, 73745.8, 1.00101
orientation: 0, 0, 0.853535, 0.521036
use_back :1
only_behind_solutions :0
time_limit :30000
shape_length :5.5
shape_width :2.75
base2back :1.5
minimum_turning_radius :9
theta_size :144
curve_weight :1.2
reverse_weight :2
lateral_goal_range :0.5
longitudinal_goal_range :2
angle_goal_range :6
obstacle_threshold :100
distance_heuristic_weight :1
*/

void set_pose(double px, double py, double pz, double ox, double oy, double oz, double ow, geometry_msgs::Pose& pose){
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  pose.orientation.w = ow;
}


int main(int argc, char** argv){
  // PLEASE RUN ROSCORE beforehand
  ros::init(argc, argv, "dummy"); // running as a ros node just because I wan to use ros::Time
  ros::NodeHandle nh;
  
  // set problem configuration
  RobotShape shape{5.5, 2.75, 1.5};
  AstarParam astar_param_{
    true, false, 30000.0, 
      shape, 9.0, 
      144, 1.2, 2.0, 0.5, 2.0, 6.0,
      100, 1.0};
  auto astar = AstarSearch(astar_param_);

  // load costmap from rosbag
  rosbag::Bag bag;
  bag.open("../bag/astar.bag", rosbag::bagmode::Read);
  vector<string> topics;
  topics.push_back("costmap");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for(rosbag::MessageInstance const m: rosbag::View(bag)){
    nav_msgs::OccupancyGrid::ConstPtr s = m.instantiate<nav_msgs::OccupancyGrid>();
    if(s != nullptr){
      astar.initializeNodes(*s);
    }
  }
  bag.close();
  std::cout << "finish setup" << std::endl; 

  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  //set_pose(3716.06, 73751.3, 0.0541427, 8.21603e-05, 5.30234e-05, 0.228649, 0.973509, start); //default
  set_pose(3714.06, 73750.3, 0.0541427, 8.21603e-05, 5.30234e-05, 0.228649, 0.973509, start);
  set_pose(3719.11, 73745.8, 1.00101, 0, 0, 0.853535, 0.521036, goal);

  const ros::WallTime begin = ros::WallTime::now();
  bool success = astar.makePlan(start, goal);
  const ros::WallTime now = ros::WallTime::now();
  const double msec = (now - begin).toSec() * 1000.0;
  if(success){
    std::cout << "plan success : " << msec << "[msec]" << std::endl; 
  }else{
    std::cout << "plan fail : " << msec << "[msec]" << std::endl; 
  }
}
