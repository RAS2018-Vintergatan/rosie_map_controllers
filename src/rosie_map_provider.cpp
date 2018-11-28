#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosie_map_updater/GetGrid.h>
#include <rosie_map_controller/CommissionRerun.h>


float pi = 3.14159265359;
float resolution;
int width = 1;
int height = 1;
int numbMarkers = 0;

float robotsize = 0.2;
static std_msgs::Int8* occGrid;

float* pointArray;

static ros::Subscriber map_sub;
static ros::Publisher grid_publisher;
static ros::Publisher wall_publisher;

ros::Time load_time;

char mapInitialized = 0;
void initializeMap(const visualization_msgs::MarkerArray msg){
	ROS_INFO("Initializing!");

	map_sub = ros::Subscriber();

	width = 480;
	height = 480;
	resolution = 0.01f;
	numbMarkers = msg.markers.size();
	//markers = (visualization_msgs::Marker*)malloc(sizeof(visualization_msgs::Marker)*numbMarkers);
	//markers = msg.markers;

	int czone = robotsize/((float)2*resolution) + 0.02/resolution; //additional extra security distance

	std::free(pointArray);
	pointArray = (float*)malloc(sizeof(float)*4*numbMarkers);
	for(int i = 0; i < 4*numbMarkers; ++i){
		pointArray[i] = 0.0f;
	}

	float minX, minY, maxX, maxY;
	minX = minY = maxX = maxY = 0;
	for(int k = 0; k < numbMarkers; ++k){
		//Extract point data
		std::string pointsText = msg.markers[k].text;
		std::stringstream ss(pointsText);
		float sX, sY, eX, eY;
		ss>>sX;
		ss>>sY;
		ss>>eX;
		ss>>eY;

		//Set point data on every 4th index
		pointArray[k<<2]=sX;
		pointArray[(k<<2)+1]=sY;
		pointArray[(k<<2)+2]=eX;
		pointArray[(k<<2)+3]=eY;

		if(sX < minX){
			minX = sX;
		}
		if(sX > maxX){
			maxX = sX;
		}
		if(eX < minX){
			minX = eX;
		}
		if(eX > maxX){
			maxX = eX;
		}
		if(sY < minY){
			minY = sY;
		}
		if(sY > maxY){
			maxY = sY;
		}
		if(eY < minY){
			minY = eY;
		}
		if(eY > maxY){
			maxY = eY;
		}
		ROS_INFO("sX: %f, sY: %f, eX: %f, eY: %f", sX, sY, eX, eY);
	}

	float offsetX = minX;
	float offsetY = minY;

	width = (maxX - minX)/resolution;
	height = (maxY - minY)/resolution;

	ROS_INFO("minX: %f, minY: %f, maxX: %f, maxY: %f, offsetX: %f, offsetY: %f", minX, minY, maxX, maxY, offsetX, offsetY);
	ROS_INFO("width: %d, height: %d", width, height);

	occGrid = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*width*height); //Int8 representation, no matrix
	for(int o = 0; o < width*height; ++o){
		occGrid[o].data = 0;
	}
	for(int k = 0; k < numbMarkers; ++k){
		float x1 = (pointArray[k<<2]) - offsetX;
		float y1 = (pointArray[(k<<2)+1]) - offsetY;
		float x2 = (pointArray[(k<<2)+2]) - offsetX;
		float y2 = (pointArray[(k<<2)+3]) - offsetY;

		float diffX = (x2-x1);
		float diffY = (y2-y1);

		float wallDist = sqrt((diffY*diffY)+(diffX*diffX));
		if(wallDist == 0){
			continue;
		}

		for(float d = 0.0; d <= wallDist; d+=resolution){
			int px = (int)((d*((x2-x1)/wallDist)+x1)/resolution);
			int py = (int)((d*((y2-y1)/wallDist)+y1)/resolution);

			occGrid[py*width+px].data = 125;			

			for(int y = -czone; y <= czone; y++){
				for(int x = -czone; x <= czone; x++){
					if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
						if(occGrid[(py+y)*width+px+x].data != 125){
							occGrid[(py+y)*width+px+x].data = 124;
						}
					}
				}
			}
		}
	}
	ROS_INFO("occGrid set!");
	mapInitialized = 1;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_provider");

    ros::NodeHandle n;
    map_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 1, initializeMap);
		grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1);
		//wall_publisher = n.advertise<std_msgs::Float32*>("/walls",1);
		ros::ServiceClient mapClient = n.serviceClient<rosie_map_updater::GetGrid>("get_updated_grid");
		ros::ServiceClient recalcClient = n.serviceClient<rosie_map_controller::CommissionRerun>("commission_rerun");
		rosie_map_controller::CommissionRerun recalcSrv;
		rosie_map_updater::GetGrid mapSrv;

    ros::Rate loop_rate(5);


	static tf::TransformBroadcaster br;

	load_time = ros::Time::now();
	int seq = 0;
	bool isUpdate = 0;
	while(ros::ok()){
		if(!mapInitialized){
		    ros::spinOnce();
	    	loop_rate.sleep();
			continue;
		}
		mapSrv.request.question = 1;
		if(mapClient.call(mapSrv)){
			if(mapSrv.response.answer){
				for(int i = 0; i<width*height;i++){
					occGrid[i].data = mapSrv.response.newGrid.data[i];
					isUpdate = 1;
				}
			}
		}
		nav_msgs::OccupancyGrid mapgrid;

		mapgrid.header.seq = seq++;
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
			mapgrid.data.push_back(occGrid[i].data);
		}

	  	grid_publisher.publish(mapgrid);

		/*std_msgs::Float32* walls = (std_msgs::Float32*)malloc(sizeof(std_msgs::Float32)*(4*numbMarkers+2));
		walls[0].data = width;
		walls[1].data = height;
		for(int i = 0; i<4*numbMarkers; ++i){
			walls[i+2].data = pointArray[i];
		}

		wall_publisher.publish(walls);*/

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		tf::Quaternion qtf;
		qtf.setRPY(0, 0, 0);
		transform.setRotation( qtf );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

		if(isUpdate){
			recalcSrv.request.command = 1;
			if(recalcClient.call(recalcSrv)){
				if(recalcSrv.response.answer){
					ROS_INFO("updated map - commissioned a path recalculation");
					isUpdate = 0;
				}
			}
		}

	    ros::spinOnce();

    	loop_rate.sleep();

    }
}
