#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <grp_navi/purePursuit.h>
#include <tf/tf.h>

geometry_msgs::Twist cmd_vel;

Vector3d robot_pose;
Vector2d goal;

void callback_goal(const geometry_msgs::PointConstPtr& goal_msgs);
void callback_state(const nav_msgs::OdometryConstPtr& odom_msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "grp_controller");
    ros::NodeHandle n;

    ros::Subscriber goal_sub = n.subscribe("/grp_navi/plan",1,callback_goal);
    ros::Subscriber odom_sub = n.subscribe("/RosAria/odom", 1, callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

    purePursuit pure_pursuit;
    while(ros::ok())
    {
        if(goal.rows() != 0){
            control pp_control = pure_pursuit.get_control(robot_pose, goal);
            setcmdvel(pp_control.v, pp_control.w);
            cmd_vel_pub.publish(cmd_vel);
        }

        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    return 0;
}

void callback_state(const nav_msgs::OdometryConstPtr& odom_msgs){
    robot_pose(0) = odom_msgs->pose.pose.position.x;
    robot_pose(1) = odom_msgs->pose.pose.position.y;
    robot_pose(2) = tf::getYaw(odom_msgs->pose.pose.orientation);

}

void callback_goal(const geometry_msgs::PointConstPtr& goal_msgs){
    goal(0) = goal_msgs->x;
    goal(1) = goal_msgs->y;
}

void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}
