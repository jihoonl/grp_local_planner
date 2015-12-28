
#include <unistd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Cholesky>
#include <nav_msgs/Odometry.h>
#include <GP/GPRClass2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <ros/package.h>
#include <time.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
MatrixXd waypoints;
int psb_idx = 0;

double res;
cv::Mat map;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

double map_size = 10; // originally x axis : (-5, 5), y axis : (0, 10)
double mat_size = 800;
double map_resize_const = 0.0; //if you increase this value, window size(now 10m) becomse smaller(10m/map_resize_const)

MatrixXd curr_path;
MatrixXd rand_paths;
MatrixXd pub_path;
double scale = 100;
double eps = 1.01;
double intv_len_const = 30;

MatrixXd pnts(4, 2);

geometry_msgs::Point lookahead;
Vector3d robot_pose;
Vector3d goal;


void generate_path();
MatrixXd sample_gppath(double unit_intv_len, double *hyp);
MatrixXd choose_path(MatrixXd rand_paths, double unit_intv_len, double *hyp);

bool isCollision(VectorXd x1, VectorXd x2);
bool isVisible(VectorXd x1);
double getdist(VectorXd p1, VectorXd p2);
void visualize();
MatrixXd linspace(double d0, double d1, int n);
MatrixXd set_waypoints();

void callback_state(const nav_msgs::OdometryConstPtr& odom_msgs);

cv_bridge::CvImagePtr cv_ptr;

int calculate_cost(VectorXd x1, VectorXd x2);
int intv_num = 7;
double old_percent_cost = 0;

double percent_cost = 0;
ros::Publisher marker_pub;

void replanning(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv){

    ros::init(argc, argv, "grp_replanning");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber map_sub;
    ros::Publisher goal_pub = n.advertise<geometry_msgs::Point>("/grp_navi/plan", 1);

    map_sub = it.subscribe("/grp_navi/map", 1, replanning);
    ros::Subscriber odom_sub = n.subscribe("/RosAria/odom", 1, callback_state);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    waypoints = set_waypoints();

    int pnt_idx = 0;
    int path_pnt_idx = 0;
    int old_pnt_idx = pnt_idx;

    goal(0) = waypoints(pnt_idx, 0);
    goal(1) = waypoints(pnt_idx, 1);

    int reach_pnt = -1;
int test = 0;

    while (ros::ok()){

        if (map.rows != 0){

            for (int i = pnt_idx; i > reach_pnt; i--){
                if (getdist(robot_pose, waypoints.row(i)) < 1.5)
                    reach_pnt++;
            }
            if (getdist(goal, robot_pose) < 0.5)
            {

                pnt_idx++;



                if (pnt_idx >= waypoints.rows())
                    break;


            }
            if (isVisible(waypoints.row(pnt_idx)) == true){

                while(pnt_idx + 1 < waypoints.rows()){
                    if( isVisible(waypoints.row(pnt_idx+1)) == true)
                        pnt_idx++;
                    else {
                        break;
                    }
                }

                goal(0) = waypoints(pnt_idx, 0);
                goal(1) = waypoints(pnt_idx, 1);
                if (pnt_idx < waypoints.rows() - 1){
                    goal(2) = atan2(waypoints(pnt_idx + 1, 1) - goal(1), waypoints(pnt_idx + 1, 0) - goal(0));
                }
                else goal(2) = 100;
            }

            else {

                VectorXd tmp_goal = waypoints.row(pnt_idx);

                double slope = atan2(-robot_pose(1) + tmp_goal(1), -robot_pose(0) + tmp_goal(0)) - robot_pose(2) + M_PI_2;
                if(slope > M_PI)
                    slope -= 2*M_PI;
                else if(slope < -M_PI)
                    slope += 2*M_PI;

                if (slope > atan2(world_y_max, world_x_max) && slope < atan2(world_y_max, world_x_min)){
                    goal(1) = world_y_max;
                    goal(0) = goal(1) / tan(slope);
                    if(goal(0) > world_x_max )
                        goal(0) = world_x_max;
                    else if (goal(0) < world_x_min)
                        goal(0) = world_x_min;

                }
                else if (slope < atan2(world_y_max, world_x_max) && slope > atan2(world_y_min, world_x_max)){
                    goal(0) = world_x_max;
                    goal(1) = goal(0) * tan(slope);
                }
                else if (slope < atan2(world_y_min, world_x_min) || slope > atan2(world_y_max, world_x_min)){

                    goal(0) = world_x_min;
                    goal(1) = goal(0) * tan(slope);

                }
                else {

                    goal(1) = world_y_min;
                    goal(0) = goal(1)/tan(slope);
                    if(goal(0) > world_x_max )
                        goal(0) = world_x_max;
                    else if (goal(0) < world_x_min)
                        goal(0) = world_x_min;

                }

				tmp_goal(0) = robot_pose(0) + goal(0)*cos(-M_PI_2 + robot_pose(2)) - goal(1)*sin(-M_PI_2 + robot_pose(2));
                tmp_goal(1) = robot_pose(1) + goal(0)*sin(-M_PI_2 + robot_pose(2)) + goal(1)*cos(-M_PI_2 + robot_pose(2));

                goal(0) = tmp_goal(0);
                goal(1) = tmp_goal(1);

                goal(2) = 100;
}

clock_t begin = clock();
            generate_path();
            clock_t end = clock();
            
            if (percent_cost < 0.8){
                while (percent_cost < 0.8){

                    if (pnt_idx > reach_pnt + 1)
                        pnt_idx--;
                    else
                        break;


                    goal(0) = waypoints(pnt_idx, 0);
                    goal(1) = waypoints(pnt_idx, 1);

                    if (pnt_idx < waypoints.rows() - 1){

                        goal(2) = atan2(waypoints(pnt_idx + 1, 1) - goal(1), waypoints(pnt_idx + 1, 0) - goal(0));
                    }
                    else goal(2) = 100;

                    clock_t begin = clock();

                    generate_path();

                    clock_t end = clock();
                    
                }
            }

        if (pub_path.rows() == 0){
            pub_path = curr_path;
            path_pnt_idx =1;
            old_percent_cost = percent_cost;
            old_pnt_idx = pnt_idx;
        }
        else if (2*path_pnt_idx > pub_path.rows()  || old_percent_cost < 0.8 || old_pnt_idx != pnt_idx){
            if(pub_path.rows() > 5 || old_pnt_idx != pnt_idx){
            pub_path = curr_path;
            path_pnt_idx = 1;
            old_percent_cost = percent_cost;
            old_pnt_idx = pnt_idx;
            }
        }
        if (pub_path.rows() != 0){

            while (getdist(robot_pose, pub_path.row(path_pnt_idx) / scale) < 0.5 && path_pnt_idx < pub_path.rows()-1){

                path_pnt_idx++;
                }
            lookahead.x = pub_path(path_pnt_idx,0) / 100.0;
            lookahead.y = pub_path(path_pnt_idx,1) / 100.0;
            goal_pub.publish(lookahead);
        }
        visualize();

}
        ros::spinOnce();
        ros::Rate(100).sleep();

    }
}


void callback_state(const nav_msgs::OdometryConstPtr& odom_msgs){
    robot_pose(0) = odom_msgs->pose.pose.position.x;
    robot_pose(1) = odom_msgs->pose.pose.position.y;
    robot_pose(2) = tf::getYaw(odom_msgs->pose.pose.orientation);

}


void replanning(const sensor_msgs::ImageConstPtr& msg){

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    map = cv_ptr->image;

    map_y_range = map.cols;
    map_x_range = map.rows;

    map_resize_const = (double)(mat_size / map_y_range);

    world_x_min = -(double)(map_size / (2 * map_resize_const));
    world_x_max = (double)(map_size / (2 * map_resize_const));
    world_y_min = -(double)(map_size/(2*map_resize_const));
    world_y_max = (double)(map_size / (2*map_resize_const));
    
    res = (double)(map_size / mat_size);
    
    map_origin_x = ceil(mat_size / map_resize_const/2.0) - 0.5;
    map_origin_y = ceil(mat_size / map_resize_const / 2.0) - 0.5;

   cv::waitKey(1);

}


void generate_path()
{

    pnts(1, 0) = scale*robot_pose(0);
    pnts(1, 1) = scale*robot_pose(1);

    pnts(0, 0) = (double)(pnts(1, 0) - intv_len_const*(eps)*cos(robot_pose(2)));
    pnts(0, 1) = (double)(pnts(1, 1) - intv_len_const*(eps)*sin(robot_pose(2)));


    pnts(2, 0) = scale*goal(0);
    pnts(2, 1) = scale*goal(1);

    if (goal(2) >= -M_PI && goal(2) <= M_PI){

        pnts(3, 0) = scale*goal(0) + intv_len_const*(eps / 2)*cos(goal(2));
        pnts(3, 1) = scale*goal(1) + intv_len_const*(eps / 2)*sin(goal(2));

    }
    else{
        pnts(3, 0) = scale*goal(0) + intv_len_const*(eps / 2)*(goal(0) - robot_pose(0)) / sqrt((goal(1) - robot_pose(1))*(goal(1) - robot_pose(1)) + (goal(0) - robot_pose(0))*(goal(0) - robot_pose(0)));

        pnts(3, 1) = scale*goal(1) + intv_len_const*(eps / 2)*(goal(1) - robot_pose(1)) / sqrt((goal(1) - robot_pose(1))*(goal(1) - robot_pose(1)) + (goal(0) - robot_pose(0))*(goal(0) - robot_pose(0)));
    }

	double max_dist = 0;
    double min_dist = 9999;

    for (int i = 0; i < pnts.rows() - 1; i++){
        int j = i + 1;

        if (max_dist<getdist(pnts.row(i), pnts.row(j)))
            max_dist = getdist(pnts.row(i), pnts.row(j));
        if (min_dist>getdist(pnts.row(i), pnts.row(j)))
            min_dist = getdist(pnts.row(i), pnts.row(j));

    }
    double sig2x = 0.75*max_dist*max_dist;
    double unit_intv_len;

    double hyp[3];

    hyp[0] = max_dist*max_dist;
    hyp[1] = sig2x;
    hyp[2] = 1e-12;

    if (scale / 20 > min_dist / 5)
        unit_intv_len = min_dist / 5;
    else
        unit_intv_len = scale / 20;

    if (unit_intv_len < intv_len_const)
        unit_intv_len = intv_len_const;

    rand_paths = sample_gppath(unit_intv_len, hyp);

    curr_path = choose_path(rand_paths, unit_intv_len, hyp);
    lookahead.x = curr_path(2, 0) / 100.0;
    lookahead.y = curr_path(2, 1) / 100.0;
}

MatrixXd choose_path(MatrixXd rand_paths, double unit_intv_len, double *hyp)
{

    psb_idx = 0;

    int psb_max_dist_path = -1;
    int psb_dist_size = ceil(rand_paths.rows() / 2);
    double psb_max_dist = 0;
    double min_total_dist = 99999999;
    int max_cost = -1;

    for (int i = 0; i < rand_paths.rows() / 2; i++)
    {
        bool collision = false;
        double total_dist = 0;
        int cost = 0;
        int minimum_cost = 255 * (intv_num + 1);
        int tmp_cost = 0;


        for (int j = 1; j < rand_paths.cols() - 1; j++)
        {
            Vector2d tmp, prev_tmp;

            tmp(0) = rand_paths(2 * i, j) / scale;
            tmp(1) = rand_paths(2 * i + 1, j) / scale;

            prev_tmp(0) = rand_paths(2 * i, j - 1) / scale;
            prev_tmp(1) = rand_paths(2 * i + 1, j - 1) / scale;

            total_dist += getdist(prev_tmp, tmp);

            tmp_cost = calculate_cost(prev_tmp, tmp);

            if (tmp_cost < minimum_cost)
                minimum_cost = tmp_cost;

            cost += minimum_cost;
        }
        
		if (max_cost < cost || (max_cost == cost && min_total_dist > total_dist)){
            max_cost = cost;
            min_total_dist = total_dist;
            psb_max_dist_path = i;

        }

    }

    percent_cost = (double)max_cost / (intv_num * 255 * (rand_paths.cols() - 2));
    std::cout << "percent_cost  : " << percent_cost << std::endl;
    
    MatrixXd minimum_path = MatrixXd(rand_paths.cols(), 2);
    minimum_path.col(0) = rand_paths.row(psb_max_dist_path * 2).transpose();
    minimum_path.col(1) = rand_paths.row(psb_max_dist_path * 2 + 1).transpose();
    return minimum_path;
}


MatrixXd sample_gppath(double unit_intv_len, double* hyp)
{
    MatrixXd rand_paths, mean_path;
    int nr_path = 100;
    int nr_pnt = pnts.rows();

    MatrixXd nr_intvs(nr_pnt - 1, 1);
    MatrixXd len_abs(nr_pnt - 1, 1);
    MatrixXd min_intvs(nr_pnt - 1, 1);

    for (int i = 0; i < nr_pnt - 1; i++)
    {
        len_abs(i, 0) = getdist(pnts.row(i), pnts.row(i + 1));

        nr_intvs(i, 0) = ceil(len_abs(i, 0) / unit_intv_len);

        if (i == 0)
            min_intvs(i, 0) = nr_intvs(i, 0);
        else
            min_intvs(i, 0) = nr_intvs(i, 0) + min_intvs(i - 1, 0);
    }

    MatrixXd test_inputs;

    test_inputs = linspace(0, len_abs.sum(), nr_intvs.sum());

    MatrixXd gp_inputs(nr_pnt, 1);

    for (int i = 0; i < nr_pnt; i++)
    {
        if (i == 0)
            gp_inputs(i, 0) = test_inputs(0, 0);
        else
        {
            int tmp_i = min_intvs(i - 1, 0);

            if (tmp_i <= 0) tmp_i++;

            gp_inputs(i, 0) = test_inputs(tmp_i - 1, 0);
        }

    }

    MatrixXd gp_outputs;
    gp_outputs = pnts;

    GPRClass GPR(gp_inputs, gp_outputs, hyp);

    mean_path = GPR.GPRmean(test_inputs);

    MatrixXd Kpost, R, cR;

    Kpost = GPR.GPRvar(test_inputs);

    double var_post = 1e-7;

    Kpost = Kpost + var_post*MatrixXd::Identity(nr_intvs.sum(), nr_intvs.sum());

    double len_path = nr_intvs.sum();

    R = (Kpost + Kpost.transpose()) / 2;

    LLT<MatrixXd> A_LLT(R);
    cR = A_LLT.matrixL().transpose();

    MatrixXd Rmtx = MatrixXd::Random(2 * nr_path, len_path);

    MatrixXd multi_tmp(Rmtx.rows(), cR.cols());

    multi_tmp = GPR.Multi(Rmtx, cR);

    rand_paths = mean_path.transpose().replicate(nr_path, 1);

    rand_paths = rand_paths + multi_tmp;

    return rand_paths;
}

double getdist(VectorXd p1, VectorXd p2){

    double result = 0;

    result = sqrt((p1(0) - p2(0))*(p1(0) - p2(0)) + (p1(1) - p2(1))*(p1(1) - p2(1)));

    return result;

}


MatrixXd linspace(double d0, double d1, int n)
{
    MatrixXd vector(n, 1);

    double passo;

    passo = (double)((d1 - d0) / (n - 1));

    for (int i = 0; i <= n - 2; ++i)
    {
        vector(i, 0) = (d0 + (i*passo));
    }
    vector(n - 1, 0) = d1;
    return vector;
}


bool isCollision(VectorXd x1, VectorXd x2)
{
    int threshold = 100;

    Vector2d tmpx1, tmpx2, tmp;

    tmpx1(0) = x1(0) - robot_pose(0);
    tmpx1(1) = x1(1) - robot_pose(1);

    tmpx2(0) = x2(0) - robot_pose(0);
    tmpx2(1) = x2(1) - robot_pose(1);

    tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
    tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
    tmpx1 = tmp;

    tmp(0) = tmpx2(0)*cos(-robot_pose(2) + M_PI) - tmpx2(1)*sin(-robot_pose(2) + M_PI);
    tmp(1) = tmpx2(0)*sin(-robot_pose(2) + M_PI) + tmpx2(1)*cos(-robot_pose(2) + M_PI);
    tmpx2 = tmp;

    int map_x1 = (int)(tmpx1(0) / res + map_origin_x);
    int map_y1 = (int)(tmpx1(1) / res + map_origin_y);
    int map_x2 = (int)(tmpx2(0) / res + map_origin_x);
    int map_y2 = (int)(tmpx2(1) / res + map_origin_y);
    bool isCol = false;

    int minx = cv::min(map_x1, map_x2);
    int maxx = cv::max(map_x1, map_x2);
    int miny = cv::min(map_y1, map_y2);
    int maxy = cv::max(map_y1, map_y2);

    int i, j;

    for (j = miny; j < maxy + 1; j++)
    {
        for (i = minx; i < maxx + 1; i++)
        {
            if (map.at<uchar>(i, j) < threshold) isCol = true;
        }
    }

    return isCol;

}

int calculate_cost(VectorXd x1, VectorXd x2) {


    int cost = 0;
    int minimum_cost = 255;

    Vector2d tmpx1, tmpx2, tmp;

    tmpx1(0) = x1(0) - robot_pose(0);
    tmpx1(1) = x1(1) - robot_pose(1);

    tmpx2(0) = x2(0) - robot_pose(0);
    tmpx2(1) = x2(1) - robot_pose(1);

    tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
    tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
    tmpx1 = tmp;

    tmp(0) = tmpx2(0)*cos(-robot_pose(2) + M_PI) - tmpx2(1)*sin(-robot_pose(2) + M_PI);
    tmp(1) = tmpx2(0)*sin(-robot_pose(2) + M_PI) + tmpx2(1)*cos(-robot_pose(2) + M_PI);
    tmpx2 = tmp;

    for (int i = 0; i < intv_num; i++){

        double x, y;

        x = tmpx1(0) + (tmpx2(0) - tmpx1(0))*(i + 1) / intv_num;
        y = tmpx1(1) + (tmpx2(1) - tmpx1(1))*(i + 1) / intv_num;

        int map_x = (int)(x / res + map_origin_x);
        int map_y = (int)(y / res + map_origin_y);

        if(map.at<uchar>(map_x, map_y) < minimum_cost)
            minimum_cost = map.at<uchar>(map_x, map_y);

        cost += minimum_cost;

    }

    return cost;
}



bool isVisible(VectorXd x1){

    Vector2d tmp, tmp1;

    tmp(0) = x1(0) - robot_pose(0);
    tmp(1) = x1(1) - robot_pose(1);

    tmp1(0) = tmp(0)*cos(-robot_pose(2) + M_PI_2) - tmp(1)*sin(-robot_pose(2) + M_PI_2);
    tmp1(1) = tmp(0)*sin(-robot_pose(2) + M_PI_2) + tmp(1)*cos(-robot_pose(2) + M_PI_2);

    bool isVis = false;

    if (tmp1(0) < world_x_max && tmp1(0) > world_x_min && tmp1(1) < world_y_max && tmp1(1) > world_y_min)
        isVis = true;

    return isVis;

}


void visualize(){

    int thickness = 1;
    int lineType = 8;
    double Res = 1;
    cv::Point x1, x2;
    double radius = 6;

    cv::Mat imgResult;
    cv::cvtColor(map, imgResult, CV_GRAY2BGR);

    Vector2d tmpx1, tmpx2, tmp;
    tmpx1(0) = pnts(2, 0) / 100 - robot_pose(0);
    tmpx1(1) = pnts(2, 1) / 2 / 100 - robot_pose(1);

    tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
    tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
    tmpx1 = tmp;

    cv::circle(imgResult, cv::Point((int)(Res*(map_origin_y)), (int)(Res*(map_origin_x))), radius, cv::Scalar(0, 0, 0), CV_FILLED);



    // for rviz visualization - kj
    // finishing initialize and curr_path's points push_back and publish.
    visualization_msgs::Marker curr_path_rviz, curr_path_pnts;
    curr_path_rviz.header.frame_id = curr_path_pnts.header.frame_id = "RosAria/odom";
    curr_path_rviz.header.seq = curr_path_pnts.header.seq = 0;
    curr_path_rviz.header.stamp = curr_path_pnts.header.stamp = ros::Time::now();

    curr_path_rviz.type =visualization_msgs::Marker::LINE_STRIP;
    curr_path_pnts.type = visualization_msgs::Marker::POINTS;
    curr_path_rviz.ns = curr_path_pnts.ns = "points_and_lines";
    curr_path_rviz.action = curr_path_pnts.action = visualization_msgs::Marker::ADD;
    curr_path_rviz.pose.orientation.w = curr_path_pnts.pose.orientation.w = 1.0;
    curr_path_pnts.id = 0;
    curr_path_rviz.id = 1;

    curr_path_rviz.scale.x = 0.05;
    curr_path_pnts.scale.x = 0.1;
    curr_path_pnts.scale.y = 0.1;

    curr_path_rviz.color.b = 1.0;
    curr_path_rviz.color.a = 1.0;

    curr_path_pnts.color.g = 1.0;
    curr_path_pnts.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = 1;



    for (int i = 1; i < curr_path.rows(); i++) {

        for (int j = 0; j < rand_paths.rows(); j = j + 2)
        {

            tmpx1(0) = rand_paths(j, i - 1) - robot_pose(0) * 100;
            tmpx1(1) = rand_paths(j + 1, i - 1) - robot_pose(1) * 100;

            tmpx2(0) = rand_paths(j, i) - robot_pose(0) * 100;
            tmpx2(1) = rand_paths(j + 1, i) - robot_pose(1) * 100;

            tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
            tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
            tmpx1 = tmp;

            tmp(0) = tmpx2(0)*cos(-robot_pose(2) + M_PI) - tmpx2(1)*sin(-robot_pose(2) + M_PI);
            tmp(1) = tmpx2(0)*sin(-robot_pose(2) + M_PI) + tmpx2(1)*cos(-robot_pose(2) + M_PI);
            tmpx2 = tmp;

            x1 = cv::Point((int)(Res*(tmpx1(1) / res / 100 + map_origin_y)), (int)(Res*(tmpx1(0) / res / 100 + map_origin_x)));
            x2 = cv::Point((int)(Res*(tmpx2(1) / res / 100 + map_origin_y)), (int)(Res*(tmpx2(0) / res / 100 + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(150, 0, 0), thickness, lineType);
        }
	}

    for(int i = 1; i <pub_path.rows(); i ++){
        
		tmpx1(0) = pub_path(i, 0) - robot_pose(0) * 100;
        tmpx1(1) = pub_path(i, 1) - robot_pose(1) * 100;

        tmpx2(0) = pub_path(i - 1, 0) - robot_pose(0) * 100;
        tmpx2(1) = pub_path(i - 1, 1) - robot_pose(1) * 100;

        tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
        tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
        tmpx1 = tmp;

        tmp(0) = tmpx2(0)*cos(-robot_pose(2) + M_PI) - tmpx2(1)*sin(-robot_pose(2) + M_PI);
        tmp(1) = tmpx2(0)*sin(-robot_pose(2) + M_PI) + tmpx2(1)*cos(-robot_pose(2) + M_PI);
        tmpx2 = tmp;

        x1 = cv::Point((int)(Res*(tmpx1(1) / res / 100 + map_origin_y)), (int)(Res*(tmpx1(0) / res / 100 + map_origin_x)));
        x2 = cv::Point((int)(Res*(tmpx2(1) / res / 100 + map_origin_y)), (int)(Res*(tmpx2(0) / res / 100 + map_origin_x)));
        cv::circle(imgResult, x2, radius, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::circle(imgResult, x1, radius, cv::Scalar(0, 0, 255), CV_FILLED);

        tmpx1(0) = pub_path(i, 0) - robot_pose(0) * 100;
        tmpx1(1) = pub_path(i, 1) - robot_pose(1) * 100;

        tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI_2) - tmpx1(1)*sin(-robot_pose(2) + M_PI_2);
        tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI_2) + tmpx1(1)*cos(-robot_pose(2) + M_PI_2);
        tmpx1 = tmp;

        p.x = tmpx1(0)/100; p.y = tmpx1(1)/100;
        curr_path_rviz.points.push_back(p);
        curr_path_pnts.points.push_back(p);

        cv::line(imgResult, x1, x2, cv::Scalar(0, 0, 255), thickness, lineType);

    }
    for (int i = 0; i < waypoints.rows(); i++){
        tmpx1(0) = waypoints(i, 0) - robot_pose(0);
        tmpx1(1) = waypoints(i, 1) - robot_pose(1);

        tmp(0) = tmpx1(0)*cos(-robot_pose(2) + M_PI) - tmpx1(1)*sin(-robot_pose(2) + M_PI);
        tmp(1) = tmpx1(0)*sin(-robot_pose(2) + M_PI) + tmpx1(1)*cos(-robot_pose(2) + M_PI);
        tmpx1 = tmp;

        cv::circle(imgResult, cv::Point((int)(Res*(tmpx1(1) / res + map_origin_y)), (int)(Res*(tmpx1(0) / res + map_origin_x))), radius * 2, cv::Scalar(0, 255, 0), CV_FILLED);

    }
    marker_pub.publish(curr_path_rviz);
    marker_pub.publish(curr_path_pnts);

    cv::namedWindow("PLANNING", 0);

    cv::imshow("PLANNING", imgResult);

    cv::waitKey(1);

}


MatrixXd set_waypoints()
{
    MatrixXd waypoint_candid(0, 2);

    std::ifstream myfile;

    myfile.open("/home/scarab1/catkin_ws/src/grp_navi/src/waypnts.txt", std::ios::app);

    double x, y;

    if (myfile.is_open()){

        while (myfile >> x >> y) {

            MatrixXd tmp(waypoint_candid.rows() + 1, waypoint_candid.cols());

            for (int i = 0; i < waypoint_candid.rows(); i++)
                tmp.row(i) = waypoint_candid.row(i);

            waypoint_candid = tmp;

            waypoint_candid(waypoint_candid.rows() - 1, 0) = x;
            waypoint_candid(waypoint_candid.rows() - 1, 1) = y;

        }

    }

    myfile.close();

    return waypoint_candid;

}
