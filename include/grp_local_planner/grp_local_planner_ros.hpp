/*
  grp_local_planner_ros.hpp
  License : https://github.com/jihoonl/grp_local_planner/blob/license/LICENSE
  Author : Jihoon Lee
*/

#ifndef _GRP_LOCAL_PLANNER_ROS_HPP_
#define _GRP_LOCAL_PLANNER_ROS_HPP_

// ROS Dependency
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

// Message dependency
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// Planner Dependency
// TODO : #include "grp_local_planner/grp_local_planner.hpp"

namespace grp_local_planner
{
/**
 * @class GrpLocalPlannerROS
 * @brief Implements ROS navigation stack local planner plugin of Gaussian Random Path Local Planner
 */
class GrpLocalPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Default constructor
   */
  GrpLocalPlannerROS();

  /**
   * @brief Default Destructor
   */
  ~GrpLocalPlannerROS();

  /**
   * @brief  Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);


  /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
    * @brief  Check if the goal pose has been achieved
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();
protected:
private:
  costmap_2d::Costmap2DROS* costmap_ros_; /// Costmap ROS from nav_core
  tf::TransformListener* tf_;              /// transform listener
//  GrpLocalPlanner planner;                /// actual planner

  std::string name_;                      /// given instance name
};
}

#endif
