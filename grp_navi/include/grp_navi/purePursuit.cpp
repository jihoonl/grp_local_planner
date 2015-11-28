#include <grp_navi/purePursuit.h>
#include <iostream>

purePursuit::purePursuit(){

}

control purePursuit::get_control(Vector3d robot_pose, Vector2d goal){

    Vector2d robot_point;
    robot_point(0) = robot_pose(0);
    robot_point(1) = robot_pose(1);
    Vector2d pursuit_point = goal - robot_point;
    double dist = pursuit_point.norm();
    std::cout<<"dist:"<<dist<<std::endl;
    if(std::sqrt((robot_point - goal).dot(robot_point - goal)) > 0.5){
        pursuit_point = 0.5*pursuit_point.normalized();
    }

    double theta =std::atan2(pursuit_point(1), pursuit_point(0)) - robot_pose(2);
    if (theta > M_PI)
        theta = theta - 2 * M_PI;
    else if (theta < -M_PI)
        theta = theta + 2 * M_PI;

    double r = pursuit_point.norm()/(cos(M_PI_2-robot_pose(2))*pursuit_point(0) - sin(M_PI_2-robot_pose(2))*pursuit_point(1));


    if(dist > 1.0){
        ctrl.v = 0.3;
    }else if(dist <= 1.0){
        ctrl.v = 0.15;
    }else if(dist <= 0.5){
        ctrl.v = 0.1;
    }
    ctrl.w = ctrl.v / r;

    if(dist < 0.1){
        ctrl.v = 0.0;
        ctrl.w = 0.0;
    }

    return ctrl;
}
