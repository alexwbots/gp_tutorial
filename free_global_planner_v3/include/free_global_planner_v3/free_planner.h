#ifndef _PLANNER_H
#define _PLANNER_H

#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>

namespace free_global_planner_v3 {

  class GlobalPlanner : public nav_core::BaseGlobalPlanner {

    public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        ~GlobalPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                      double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        bool algorithm(double start_x, double start_y, double goal_x, double goal_y, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    protected:

        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        bool initialized_;
        ros::Publisher plan_pub_;
        bool print_data_;

    private:

        double default_tolerance_;
        bool worldToMap(double wx, double wy, double& mx, double& my);
        float convert_offset_;
        void mapToWorld(double mx, double my, double& wx, double& wy);
  };

}
#endif
