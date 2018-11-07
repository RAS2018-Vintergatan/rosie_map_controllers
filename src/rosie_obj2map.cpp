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
float resolution;
int width = 1;
int height = 1;

float robotsize = 0.2;
static std_msgs::Int8 *occGrid;

static ros::Subscriber map_sub;
static ros::Subscriber obj_sub;
static ros::Subscriber bat_sub;
static ros::Publisher grid_publisher;


visualization_msgs::Marker objects[10];
visualization_msgs::Marker batteries[5];
ros::Time load_time;

void objCallback(const visualization_msgs::MarkerArray msg){
		int posx[] = { 0.10, 0.50, 1.00, 4.00, 0.35, 0.80, 2.50, 3.00, 2.00, 0.10 };
		int posy[] = { 0.80, 2.00, 1.00, 0.80, 3.00, 4.00, 4.50, 2.50, 0.10, 4.00}; //assume already sorted markers
		int orient[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 120.0};
		for(int i=0;i<10;++i){
			visualization_msgs::Marker testObj;
			testObj.header.seq = 0;
			testObj.header.stamp = ros::Time::now();
			testObj.header.frame_id = 0;
			testObj.ns = "objects";
			testObj.id = i;
			testObj.type = "some";
			testObj.action = 0; //or 2 for deleting an object
			testObj.pose.position.x = posx[i]/resolution;
			testObj.pose.position.y = posy[i]/resolution;
			testObj.pose.position.z = 0.0;
			testObj.pose.orientation.x = 0.0;
			testObj.pose.orientation.y = 0.0;
			testObj.pose.orientation.z = orient[i];
			testObj.pose.orientation.w = 1;
			testObj.scale.x = 0.05;
			testObj.scale.y = 0.05;
			testObj.scale.z = 0.05;
			testObj.color.r = 1.0;
			testObj.color.b = 0.0;
			testObj.color.g = 0.0;
			testObj.color.a = 1.0;
			testObj.lifetime = 0;
			testObj.frame_locked = false;
			objects[i] = testObj[i];
		}
}

void batCallback(const visualization_msgs::MarkerArray msg){
	int posx[] = { 0.80, 2.50};
	int posy[] = { 0.80, 2.00}; //assume already sorted markers
	int orient[] = {10.0, 20.0};
	for(int i=0;i<10;++i){
		visualization_msgs::Marker testObj;
		testObj.header.seq = 0;
		testObj.header.stamp = ros::Time::now();
		testObj.header.frame_id = 0;
		testObj.ns = "battery";
		testObj.id = i;
		testObj.type = "battery";
		testObj.action = 0; //or 2 for deleting an object
		testObj.pose.position.x = posx[i]/resolution;
		testObj.pose.position.y = posy[i]/resolution;
		testObj.pose.position.z = 0.0;
		testObj.pose.orientation.x = 0.0;
		testObj.pose.orientation.y = 0.0;
		testObj.pose.orientation.z = orient[i];
		testObj.pose.orientation.w = 1;
		testObj.scale.x = 0.05/resolution;
		testObj.scale.y = 0.05/resolution;
		testObj.scale.z = 0.05/resolution;
		testObj.color.r = 1.0;
		testObj.color.b = 0.0;
		testObj.color.g = 0.0;
		testObj.color.a = 1.0;
		testObj.lifetime = 0;
		testObj.frame_locked = false;
		batteries[i] = testObj[i];
	}
}

char mapInitialized = 0;
void initializeMap(const nav_msgs::OccupancyGrid msg){
	ROS_INFO("Initializing!");

	map_sub = ros::Subscriber();

	width = msg.info.width;
	height = msg.info.height;
	resolution = msg.info.resolution;

	int czone = robotsize/((float)2*resolution) + 0.02/resolution; //additional extra security distance

	occGrid = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*width*height); //Int8 representation, no matrix
	for(int o = 0; o < width*height; ++o){
		occGrid[o].data = 0;
	}

	for(int k = 0; k < 20; ++k){
		int px = (int) objects[k].pose.position.x;
		int py = (int) objects[k].pose.position.y;
			for(int y = -czone; y <= czone; y++){
				for(int x = -czone; x <= czone; x++){
					if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
						occGrid[(py+y)*width+px+x].data = 125;
					}
				}
			}
		}
		for(int k = 0; k < 5; ++k){
			int px = (int) batteries[k].pose.position.x;
			int py = (int) batteries[k].pose.position.y;
				for(int y = -czone; y <= czone; y++){
					for(int x = -czone; x <= czone; x++){
						if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
							occGrid[(py+y)*width+px+x].data = -125;
						}
					}
				}
			}
	}
	ROS_INFO("occGrid set!");
	mapInitialized = 1;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_obj2map");

    ros::NodeHandle n;
	  map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1, initializeMap)
		obj_sub = n.subscribe<visualization_msgs::MarkerArray>("/visualization_marker",1, objCallback);
		bat_sub = n.subscribe<visualization_msgs::MarkerArray>("/visualization_marker_battery",1, batCallback);
		grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_obj_grid",1);

    ros::Rate loop_rate(5);

	static tf::TransformBroadcaster br;

	load_time = ros::Time::now();
	int seq = 0;
	while(ros::ok()){
		if(!mapInitialized){
		    ros::spinOnce();
	    	loop_rate.sleep();
			continue;
		}

		nav_msgs::OccupancyGrid mapgrid;

		mapgrid.header.seq = seq++;
		mapgrid.header.stamp = ros::Time::now();
		mapgrid.header.frame_id = "objects";

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
			mapgrid.data.push_back(occGrid[i].data);
		}

	    grid_publisher.publish(mapgrid);

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		tf::Quaternion qtf;
		qtf.setRPY(0, 0, 0);
		transform.setRotation( qtf );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "objects"));

	    ros::spinOnce();

    	loop_rate.sleep();

    }
}
