#ifndef _PLANNER_H
#define _PLANNER_H

#define POT_HIGH 1.0e10                   // Unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>        // Class that will be used by the path planner as input map
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h> // Is used to import the interface nav_core::BaseGlobal Planner
#include <nav_msgs/GetPlan.h>

namespace free_global_planner_v1 {

  class GlobalPlanner : public nav_core::BaseGlobalPlanner {

    public:

        // Constructors are special class members which are called by the compiler every time an object of that class is instantiated.
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        // Destructor is an instance member function which is invoked automatically whenever an object is going to be destroyed.
        ~GlobalPlanner();

        // The initialize method is use to initialize the object for the BaseGlobalPlanner, which initializes the costmap, 
        // that is the map that will be used for planning (costmap_ros), and the name of the planner (name).
        // The first initialize method is to call the second initialize method which need one more parameter.
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        // The makeplan method is use to initialize the object.
        // The first initialize method is to call the second initialize method which need one more parameter.
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                      double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

        // This plan will be automatically published through the plugin as a topic.
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        // The artificial intelligence algorithm for informe search
        bool algorithm(double start_x, double start_y, double goal_x, double goal_y, const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& path);

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
  };

}
#endif
