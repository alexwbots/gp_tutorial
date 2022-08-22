#include <free_global_planner_v1/free_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(free_global_planner_v1::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace free_global_planner_v1 {


  GlobalPlanner::GlobalPlanner() :
    costmap_(NULL), initialized_(false) {
  }


  // In the constructor we start the initialize method
  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
    costmap_(NULL), initialized_(false) {
      initialize(name, costmap, frame_id);
  }


  GlobalPlanner::~GlobalPlanner() {
    // Don't destroy anything for now
  }


  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  
  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {	// initialized_ start with the false value in line 13
    
      ros::NodeHandle private_nh("~/" + name);

      printf("[Initialized] name: %s\n", name.c_str());		// name: GlobalPlanner
      printf("[Initialized] frame_id: %s\n", frame_id.c_str());	// frame_id: map

      costmap_ = costmap;	// Temporal copy for the costmap
      frame_id_ = frame_id;

      unsigned int cx = costmap_->getSizeInCellsX(), cy = costmap_->getSizeInCellsY();
      printf("[Initialized] SizeInCellsX: %d and SizeInCellsY: %d \n", cx, cy);	// cx = 384 and cy = 384

      convert_offset_ = 0.5;	// Parameters used in the plan

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);	// Publish a dummy plan in this example

    } else { 
    
      ROS_WARN("This free planner has already been initialized, doing nothing!");
    }
    
    default_tolerance_ = 0.0;
    initialized_ = true;
    print_data_ = true;
  }


  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
  }


  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {

    if (!initialized_) {
      ROS_ERROR("This planner has not been initialized yet!");
      return false;
    }

    plan.clear();

    ros::NodeHandle nh;
    std::string global_frame = frame_id_;

    if(print_data_) {
      printf("[MakePlan] Global frame: %s\n", global_frame.c_str());			// Global frame: map
      printf("[MakePlan] Goal pose frame: %s\n", goal.header.frame_id.c_str());	// Goal pose frame: map
      printf("[MakePlan] Start pose frame: %s\n", start.header.frame_id.c_str());	// Start pose frame: map
    }

    double start_x, start_y;

    double world_x_tmp = start.pose.position.x; 		// world_x_tmp: 
    double world_y_tmp = start.pose.position.y;		// world_y_tmp: 
    worldToMap(world_x_tmp, world_y_tmp, start_x, start_y);	// Convert position from the map

    if(print_data_) {
      printf("[MakePlan] Map start_x: %f and start_y: %f | World start_x: %f and start_y: %f\n", start_x, start_y, world_x_tmp, world_y_tmp);
    }	// Map start_x: 160.535415 and start_y: 190.017577 | World start_x: -1.948229 and start_y: -0.474121

    double goal_x, goal_y;
    
    world_x_tmp = goal.pose.position.x;
    world_y_tmp = goal.pose.position.y;
    worldToMap(world_x_tmp, world_y_tmp, goal_x, goal_y);

    if(print_data_) {
      printf("[MakePlan] Map goal_x: %f and goal_y: %f | World goal_x: %f and goal_y: %f\n", goal_x, goal_y, world_x_tmp, world_y_tmp);
    }	// Map goal_x: 177.799997 and goal_y: 188.700005 | World goal_x: -1.085000 and goal_y: -0.540000

    int sizexcell = costmap_->getSizeInCellsX(), sizeycell = costmap_->getSizeInCellsY();

    // Look for the answer of the IA algorithm for search, the search just give you True value, it also should return the plan
    if (algorithm(start_x, start_y, goal_x, goal_y, goal, plan)) {

      geometry_msgs::PoseStamped goal_ = goal;
      goal_.header.stamp = ros::Time::now();    // The goal we push on has the same timestamp as the rest of the plan
      plan.push_back(goal_);			  // The plan comes empty plan

    } else {
      ROS_ERROR("Failed to get a plan!");
    }

    publishPlan(plan);		// Publish in the topic "plan"
    print_data_ = false;

    return false;
  }


  bool GlobalPlanner::algorithm(double start_x, double start_y, double goal_x, double goal_y, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path) {
    
    return true;
  }


  void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet!");
        return;
    }

    nav_msgs::Path gui_path;    		// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
    gui_path.poses.resize(path.size());	// the path variable come as input

    if(print_data_) {
      printf("[publishPlan] Plan size: %lu\n", path.size());	// path.size(): 1
    }

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
        if(print_data_) {
            printf("[publishPlan] path[%d]: %d\n", path.size(), path[i]);	// path[1]: 0.0
        }
    }

    plan_pub_.publish(gui_path);	// Publish in the plan topic
  }

}


  bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
   
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if(print_data_) {
      printf("[worldToMap] Get resolution: %f\n", resolution);	// Resolution: 0.05 meters/pixel
      printf("[worldToMap] Get X origin: %f and Get Y origin: %f \n", origin_x, origin_x);	
    }									// X origin: -10 & Y origin:-10

    if (wx < origin_x || wy < origin_y) {
      return false;
    }

    mx = (wx - origin_x) / resolution - convert_offset_;	// mx = (-2.0-(-10.0))/0.05
    my = (wy - origin_y) / resolution - convert_offset_;	// my = (-0.5-(-10.0))/0.05

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
      return true;
    }

    return false;
  }

