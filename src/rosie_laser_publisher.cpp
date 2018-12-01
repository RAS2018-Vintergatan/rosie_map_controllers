#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <math.h>
#include <iostream>
#include <pcl_ros/transforms.h>

#define PI 3.14159265359

std::vector<float> ranges(360,0);
std::vector<float> intensities(360,0);
float angle_increment;
float time_increment;
float scan_time;
float angle_min;
float angle_max;
float range_min;
float range_max;
std::vector<float> ranges_x(360,0);
std::vector<float> ranges_y(360,0);

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    ranges = msg->ranges;
    angle_increment = msg->angle_increment;
    time_increment = msg->time_increment;
    scan_time = msg->scan_time;
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    range_min = msg->range_min;
    range_max = msg->range_max;
    intensities = msg->intensities;

    for(int i=0; i<=359; i++){
	ranges_x[i] = ranges[i]*cos(-1.0*((double)(i)/180.0)*PI);
	ranges_y[i] = ranges[i]*sin(((double)(i)/180.0)*PI);
    }

    tf::Transform transform;
    static tf::TransformBroadcaster br;
    transform.setOrigin( tf::Vector3(-0.01, 0, 0.17) );
    tf::Quaternion qtf;
    qtf.setRPY(0, 0, 0);
    transform.setRotation( qtf );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser_frame"));
/*
  std::cout << "D-Values";
  std::copy(ranges.begin(), ranges.end(), std::ostream_iterator<float>(std::cout, " "));
  std::cout << std::endl;
  std::cout << "X-Values";
  std::copy(ranges_x.begin(), ranges_x.end(), std::ostream_iterator<float>(std::cout, " "));
  std::cout << std::endl;
  std::cout << "Y-Values";
  std::copy(ranges_y.begin(), ranges_y.end(), std::ostream_iterator<float>(std::cout, " "));
  std::cout << std::endl;
*/

}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser_publisher");

    ros::NodeHandle n;

    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidarCallback);
    ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud>("/my_cloud",100);

    ros::Rate loop_rate(10);

    tf::TransformListener listener;

    while(ros::ok()){
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud buffer;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "laser_frame";
    cloud.points.resize(360);

    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(360);
    for(unsigned int i = 0; i<360; i++){
	cloud.points[i].z = 0;
    	cloud.points[i].x = ranges_x[i];
    	cloud.points[i].y = ranges_y[i];
        cloud.channels[0].values[i] = intensities[i];
    }

   // listener.lookupTransform("my_cloud", "laser_frame", ros::Time(0),(-0.125,0,0.19,1));
    //pcl_ros::transformPointCloud(cloud, buffer, (-0.125,0,0.19,1));

    point_cloud_publisher.publish(cloud);

    ros::spinOnce();
    loop_rate.sleep();

    }
}
