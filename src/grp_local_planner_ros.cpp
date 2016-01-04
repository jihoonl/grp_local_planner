/*
  grp_local_planner_ros.cpp
  License : https://github.com/jihoonl/grp_local_planner/blob/license/LICENSE
  Author : Jihoon Lee
*/

#include "grp_local_planner/grp_local_planner_ros.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(grp_local_planner, GrpLocalPlannerROS, grp_local_planner::GrpLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace grp_local_planner {

GrpLocalPlannerROS::GrpLocalPlannerROS()
{
}

GrpLocalPlannerROS::~GrpLocalPlannerROS()
{
}

void GrpLocalPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
}


bool GrpLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  return false;
}


bool GrpLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  return false;
}

bool GrpLocalPlannerROS::isGoalReached()
{
  return false;
}

}

