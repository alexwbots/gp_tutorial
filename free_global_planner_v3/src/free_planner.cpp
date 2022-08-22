#include <free_global_planner_v3/free_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(free_global_planner_v3::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace free_global_planner_v3 {


  GlobalPlanner::GlobalPlanner() :
    costmap_(NULL), initialized_(false) {
  }


  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
    costmap_(NULL), initialized_(false) {

      initialize(name, costmap, frame_id);
  }


  GlobalPlanner::~GlobalPlanner() {
    //Don't destroy anything for now
  }


  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }


  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
    
      ros::NodeHandle private_nh("~/" + name);

      printf("[Initialized] name: %s\n", name.c_str());
      printf("[Initialized] frame_id: %s\n", frame_id.c_str());

      costmap_ = costmap;    // Temporal copy for the costmap
      frame_id_ = frame_id;

      unsigned int cx = costmap_->getSizeInCellsX(), cy = costmap_->getSizeInCellsY();

      convert_offset_ = 0.0;    // Parameters used in the plan

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      printf("Width of the map in cells is: %d\n", costmap_->getSizeInCellsX());
      printf("Height of the map in cells is: %d\n", costmap_->getSizeInCellsY());
      printf("The size of the cells in meters X is: %f\n", costmap_->getSizeInMetersX());
      printf("The size of the cells in meters Y is: %f\n", costmap_->getSizeInMetersY());
      printf("The Free Space value: %d\n", costmap_2d::FREE_SPACE);
      printf("The Lethal Space cost: %d\n",costmap_2d::LETHAL_OBSTACLE);
      printf("The Inscribed Inflated Obstacle cost: %d\n",costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      printf("The no Information cost: %d\n",costmap_2d::NO_INFORMATION);

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
      printf("[MakePlan] Global frame: %s\n", global_frame.c_str());
      printf("[MakePlan] Goal pose frame: %s\n", goal.header.frame_id.c_str());
      printf("[MakePlan] Start pose frame: %s\n", start.header.frame_id.c_str());
    }

    double start_x, start_y, goal_x, goal_y;

    double world_x_tmp = start.pose.position.x; 
    double world_y_tmp = start.pose.position.y;
    worldToMap(world_x_tmp, world_y_tmp, start_x, start_y);

    if(print_data_) {
      printf("[MakePlan] Map start_x: %f and start_y: %f | World start_x: %f and start_y: %f\n", start_x, start_y, world_x_tmp, world_y_tmp);
    }

    world_x_tmp = goal.pose.position.x;
    world_y_tmp = goal.pose.position.y;
    worldToMap(world_x_tmp, world_y_tmp, goal_x, goal_y);

    if(print_data_) {
      printf("[MakePlan] Map goal_x: %f and goal_y: %f | World goal_x: %f and goal_y: %f\n", goal_x, goal_y, world_x_tmp, world_y_tmp);
    }

    int sizexcell = costmap_->getSizeInCellsX(), sizeycell = costmap_->getSizeInCellsY();

    if (algorithm(start_x, start_y, goal_x, goal_y, goal, plan)) {

      geometry_msgs::PoseStamped goal_ = goal;
      goal_.header.stamp = ros::Time::now();    // The goal we push on has the same timestamp as the rest of the plan
      plan.push_back(goal_);    // Add the goal to the planning path

    } else {
      ROS_ERROR("Failed to get a plan!");
    }

    publishPlan(plan);
    print_data_ = false;

    return true;
  }


  bool GlobalPlanner::algorithm(double start_x, double start_y, double goal_x, double goal_y, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    
    std::string global_frame = frame_id_;
    std::vector<std::pair<float, float>> path;
    ros::Time plan_time = ros::Time::now();

    /**********     Start finding the path, it actualize every time     **********/ 
    
    std::pair<float, float> current;
    current.first = 160; current.second = 190; path.push_back(current);
    current.first = 170; current.second = 190; path.push_back(current);
    current.first = 180; current.second = 190; path.push_back(current);
    current.first = start_x; current.second = start_y; path.push_back(current);    

    /**********     End of the find path     **********/

    printf("[algorithm] Path size : %lu\n", path.size());

    plan.clear();    //clear the plan, just in case

    for (int i = path.size() -1; i>=0; i--) {

        std::pair<float, float> point = path[i];

        double world_x, world_y;     //convert the plan to world coordinates
        mapToWorld(point.first, point.second, world_x, world_y);
        printf("[algorithm] Point number %i : world_x = %f , world_y = %f\n", i, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }

    return !plan.empty();
  }


  void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet!");
        return;
    }

    nav_msgs::Path gui_path;    //create a message for the plan
    gui_path.poses.resize(path.size());

    if(print_data_) {
      printf("[publishPlan] Plan size: %lu\n", path.size());
    }

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }


  bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
   
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if(print_data_) {
      printf("[worldToMap] Get resolution: %f\n", resolution);
      printf("[worldToMap] Get X origin: %f and Get Y origin: %f \n", origin_x, origin_x);
    }

    if (wx < origin_x || wy < origin_y) {
      return false;
    }

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
      return true;
    }

    return false;
  }


  void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
  }


}
