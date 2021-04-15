#include "astar_search/astar_search.h"

/* obtained from standard output
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

int main(){
  // set problem configuration
  RobotShape shape{5.5, 2.75, 1.5};
  AstarParam astar_param_{
    true, false, 30000.0, 
      shape, 9.0, 
      144, 1.2, 2.0, 0.5, 2.0, 6.0,
      100, 1.0};
  auto astar = AstarSearch(astar_param_);
}
