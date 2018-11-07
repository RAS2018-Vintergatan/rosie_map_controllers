#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>
#include <math.h>
#include <iostream>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

float pi = 3.14159265359;
float resolution = 0.01;
int width = 1;
int height = 1;
float win = 0.5;
const int win_cells = (int) win/resolution;

geometry_msgs::Pose occOrigin;
nav_msgs::OccupancyGrid mapgrid;
	tf::Quaternion qtf;
	tf::Transform transform;
float robotsize = 0.2;
int czone = robotsize/((float)2*resolution) + 0.02/resolution; 

nav_msgs::OccupancyGrid occGrid;
visualization_msgs::Marker *markers;
sensor_msgs::PointCloud lidarPoints;
static ros::Subscriber map_sub;
static ros::Publisher grid_publisher;


char mapUpdated = 0;

ros::Time load_time;

float window[50] = { };

nav_msgs::Odometry pose;

/* float * pol2car(float length, int angle){
	float theta = angle/180.0*pi;
	float x = length*cos(theta);
	float y = length*sin(theta);
	float pos[] = { x, y };
	return pos;
}*/

void odomCallback(nav_msgs::Odometry msg){
	pose=msg;
}

/*void gridCallback(nav_msgs::OccupancyGrid msg){
	width = msg.info.width;
	height = msg.info.height;
	occOrigin = msg.info.origin;



	mapgrid.header.seq = msg.header.seq++;
	mapgrid.header.stamp = ros::Time::now();
	mapgrid.header.frame_id = "map";

	mapgrid.info.origin.position.x = 0;
	mapgrid.info.origin.position.y = 0;
	mapgrid.info.origin.position.z = 0;
	mapgrid.info.origin.orientation.x = 0;
	mapgrid.info.origin.orientation.y = 0;
	mapgrid.info.origin.orientation.z = 0;
	mapgrid.info.origin.orientation.w = 1;

	mapgrid.info.map_load_time = load_time;
	mapgrid.info.resolution = resolution;

	mapgrid.info.width = width;
	mapgrid.info.height = height;
	mapgrid.data.clear();

	for(int i = 0; i < width*height; ++i){
		mapgrid.data.push_back(msg.data[i]);
	}

	occGrid = msg;

	float px = pose.pose.pose.position.x;
	float py = pose.pose.pose.position.y;		

	for(int y = -czone; y <= czone; y++){
		for(int x = -czone; x <= czone; x++){
			if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
				if(window[(int) ((py+y)*width+px+x)] == 125){
					mapgrid.data[(int) ((py+y)*width+px+x) ] = 125;
				}					
			}
		}
	}			

 

	grid_publisher.publish(occGrid);



}*/
int mapInitialized = 0;

void lidarCallback(sensor_msgs::PointCloud msg){	
	//ROS_INFO("");

	//lidarPoints = msg;
	int size = sizeof(lidarPoints.points)*sizeof(lidarPoints.points[0]);

	for(int i = 0; i<size; ++i){
		int px = msg.points[i].x/resolution;
		int py = msg.points[i].y/resolution;
		if(abs(px) < win_cells && abs(py) < win_cells){
			window[(int) (py*2*win_cells + px)] = 125;
		}	
	}		
    

}

int seq = 0;
void mapUpdate(){
		
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_updater");


    ros::NodeHandle n;

    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",100,odomCallback);
	//Subscribe to real time LIDAR point cloud (orientation independent)
    ros::Subscriber realscan_sub = n.subscribe<sensor_msgs::PointCloud>("/scan",100,lidarCallback);
	//Subscribe to transformed LIDAR occupancy grid (Fixed to robot frame)
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::PointCloud>("/my_cloud",100,lidarCallback);
    //ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1, gridCallback);
    grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_win_grid",1);

	tf::TransformBroadcaster br;

    ros::Rate loop_rate(5);

	load_time = ros::Time::now();
	int seq = 0;	
	while(ros::ok()){
		if(!mapInitialized){
		    ros::spinOnce();
	    	loop_rate.sleep();
			continue;
		}
	    nav_msgs::OccupancyGrid windowgrid;

		windowgrid.header.seq = seq++;
		windowgrid.header.stamp = ros::Time::now();
		windowgrid.header.frame_id = "win";

		windowgrid.info.origin.position.x = 0;
		windowgrid.info.origin.position.y = 0;
		windowgrid.info.origin.position.z = 0;
		windowgrid.info.origin.orientation.x = 0;
		windowgrid.info.origin.orientation.y = 0;
		windowgrid.info.origin.orientation.z = 0;
		windowgrid.info.origin.orientation.w = 1;

		windowgrid.info.map_load_time = load_time;
		windowgrid.info.resolution = resolution;

		windowgrid.info.width = 2*win;
		windowgrid.info.height = 2*win;
		windowgrid.data.clear();
		for(int i = 0; i < width*height; ++i){
			windowgrid.data.push_back(window[i]);
		}

		grid_publisher.publish(mapgrid);
		//mapUpdate(br);
		transform.setOrigin( tf::Vector3(0, 0, 0) );
	
		qtf.setRPY(0, 0, 0);
		transform.setRotation( qtf );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "win"));	
		mapInitialized =1;
		
	    ros::spinOnce();

    	loop_rate.sleep();

    }
}
