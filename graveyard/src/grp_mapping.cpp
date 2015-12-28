
//This node generates occuapncy map from lasersacn msgs,
//publishes the map transformed to Image msg
//and visualize Occupancy map using rviz

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

double map_size = 10;			// Originally occupancy map range is 10m * 10mx and robot is located in the center
int mat_size = 800;			    // Originally occupancy map(openCV) size is 800 * 800
int grid_map_size = 300;		// Originally occupancy map(rviz) size is 300*300
double map_resize_const = 1;	// if you increase this value, window size(now 10m) becomse smaller(10m/map_resize_const)


ros::Publisher occupancygrid_pub;
void visualize();
void sensor_callback(sensor_msgs::LaserScan msgs);

int main(int argc, char** argv){

    ros::init(argc, argv, "grp_mapping");
    ros::NodeHandle n;

    ros::Subscriber sensor_sub = n.subscribe("/scan", 1, sensor_callback);
    occupancygrid_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancygrid", 1);

    image_transport::ImageTransport it_(n);
    image_transport::Publisher map_pub;
    map_pub = it_.advertise("/grp_navi/map", 1);

    map = cv::Mat::ones(ceil(mat_size / map_resize_const), ceil(mat_size / map_resize_const), CV_8UC1) * 255;

    world_x_min = -(double)(map_size / (2 * map_resize_const));
    world_x_max = (double)(map_size / (2 * map_resize_const));
    world_y_min = -(double)(map_size / (2 * map_resize_const));
    world_y_max = (double)(map_size / (2 * map_resize_const));

    res = (double)(map_size / mat_size);

    map_origin_x = ceil(mat_size / map_resize_const / 2.0) - 0.5;
    map_origin_y = ceil(mat_size / map_resize_const / 2.0) - 0.5;

    while (ros::ok())
    {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->encoding = "mono8";
        cv_ptr->image = map;
        map_pub.publish(cv_ptr->toImageMsg());
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    return 0;
}


void sensor_callback(sensor_msgs::LaserScan msgs){

    map = cv::Mat::ones(ceil(mat_size / map_resize_const), ceil(mat_size / map_resize_const), CV_8UC1) * 255;

    nav_msgs::OccupancyGrid grid_map;

    double ref = mat_size/map_resize_const/grid_map_size;

    geometry_msgs::Pose pose;

    pose.position.x = -map_size/map_resize_const/2;			// Locate the robot at the center of Occupancy grid
    pose.position.y = -map_size/map_resize_const/2;

    grid_map.header.frame_id = "RosAria/odom";
    grid_map.header.seq = 0;
    grid_map.header.stamp = ros::Time::now();

    grid_map.info.resolution = res*ref;
    grid_map.info.width = grid_map_size;
    grid_map.info.height = grid_map_size;
    grid_map.info.origin = pose;

    grid_map.data.resize(grid_map_size * grid_map_size);
    grid_map.data.assign(grid_map.data.size(), 0);		// nav_msgs::OccupancyGrid data : if occupied 100, else 0 (not using uncertain value -1 in this code)

    double x, y, mapx, mapy;


    for (int i = 0; i <= (msgs.angle_max - msgs.angle_min) / msgs.angle_increment; i++){

        if (msgs.ranges[i] < msgs.range_max && msgs.ranges[i] > msgs.range_min){

            x = -msgs.ranges[i] * cos(msgs.angle_min + msgs.angle_increment*i);
            y = -msgs.ranges[i] * sin(msgs.angle_min + msgs.angle_increment*i);

            mapx = (x / res + map_origin_x);
            mapy = (y / res + map_origin_y);

            if (mapx > 0 && mapx < ceil(mat_size / map_resize_const) && mapy > 0 && mapy < ceil(mat_size / map_resize_const)){

                cv::circle(map, cv::Point(mapy, mapx), ceil(20), 0, -1);

                mapx /= ref;
                mapy /= ref;

                if (ceil(mapy) < ceil(mat_size / map_resize_const / ref) || ceil(mapx) < ceil(mat_size / map_resize_const / ref)){

                    int tmp = (grid_map_size - ceil(mapx))*grid_map_size + ceil(mapy);

                    if (tmp < ceil(grid_map_size)*ceil(grid_map_size) && tmp >= 0)
                        grid_map.data[tmp] = 100;

                }
            }
        }
    }

    int k_size = 51;

    if (k_size % 2 == 0)
        k_size += 1;

    cv::GaussianBlur(map, map, cv::Size(k_size, k_size), 10);
   
    occupancygrid_pub.publish(grid_map);
    visualize();			// If you do not want to see map using openCV window, erase this line.

    cv::waitKey(1);

}


void visualize(){

    cv::namedWindow("Mapping", 0);
    cv::imshow("Mapping", map);
    cv::waitKey(1);

}

