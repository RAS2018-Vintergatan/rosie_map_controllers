#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>
#include <math.h>
#include <iostream>

sensor_msgs::PointCloud cloud;
geometry_msgs::Point32* staticCloud;

ros::Time load_time;

nav_msgs::OccupancyGrid occGrid;

nav_msgs::Odometry odom;


char odomGotten = 0;
char occGridGotten = 0;
char lidarGotten = 0;

/* float * pol2car(float length, int angle){
	float theta = angle/180.0*pi;
	float x = length*cos(theta);
	float y = length*sin(theta);
	float pos[] = { x, y };
	return pos;
}*/


void odomCallback(nav_msgs::Odometry msg){
	odom = msg;
	odomGotten = 1;
}

void gridCallback(nav_msgs::OccupancyGrid msg){
	occGrid = msg;
	occGridGotten = 1;
}

void lidarCallback(sensor_msgs::PointCloud msg){	
	cloud = msg;
	lidarGotten = 1;
}

int seq = 0;

int getGridX(float worldX, float resolution){
	int gridX = worldX/resolution;
	return gridX;
}

int getGridY(float worldY, float resolution){
	int gridY = worldY/resolution;
	return gridY;
}

int getOccGridValue(int x, int y){
	int gridWidth = occGrid.info.width;
	int gridHeight = occGrid.info.height;
	return occGrid.data[y*gridWidth+x];
}

long getConfigScore(int x, int y, float angle, int* grid, geometry_msgs::Point32* cloud, nav_msgs::Odometry odom,
					 float gridResolution, int searchScale, int scaledGridWidth, int scaledGridHeight){
	int robotDiffX = x-odom.pose.pose.position.x;
	int robotDiffY = y-odom.pose.pose.position.y;
	float robotAngle = odom.pose.pose.orientation.z;
	
	long currentScore = -sqrt(std::pow(robotDiffX,2)+std::pow(robotDiffY,2)) - std::abs(angle - robotAngle);
	for(int i = 0; i < 360; ++i){
		geometry_msgs::Point32 point = cloud[i];				

		geometry_msgs::Point32 transformedPoint;
		transformedPoint.x = point.x*cos(angle) + point.y*sin(angle);
		transformedPoint.y = -point.x*sin(angle) + point.y*cos(angle);

		int gridX = getGridX(x + transformedPoint.x, gridResolution);
		int gridY = getGridY(y + transformedPoint.y, gridResolution);

		if(gridX > scaledGridWidth || gridX < 0){
			continue;
		}
		if(gridY > scaledGridHeight || gridY < 0){
			continue;
		}

		int gridValue = grid[gridY*scaledGridWidth+gridX];
		if(gridValue > 0){
			currentScore += gridValue;
		}else{
			currentScore -= 1;
		}
	}
	return currentScore;
}

long getConfigScore(int x, int y, float angle, nav_msgs::OccupancyGrid grid, geometry_msgs::Point32* cloud, nav_msgs::Odometry odom,
					 float gridResolution, int searchScale, int scaledGridWidth, int scaledGridHeight){
	int robotDiffX = x-odom.pose.pose.position.x;
	int robotDiffY = y-odom.pose.pose.position.y;
	float robotAngle = odom.pose.pose.orientation.z;
	
	long currentScore = -sqrt(std::pow(robotDiffX,2)+std::pow(robotDiffY,2)) - std::abs(angle - robotAngle);
	for(int i = 0; i < 360; ++i){
		geometry_msgs::Point32 point = cloud[i];				

		geometry_msgs::Point32 transformedPoint;
		transformedPoint.x = point.x*cos(angle) + point.y*sin(angle);
		transformedPoint.y = -point.x*sin(angle) + point.y*cos(angle);

		int gridX = getGridX(x + transformedPoint.x, gridResolution);
		int gridY = getGridY(y + transformedPoint.y, gridResolution);

		if(gridX > scaledGridWidth || gridX < 0){
			continue;
		}
		if(gridY > scaledGridHeight || gridY < 0){
			continue;
		}

		int gridValue = grid.data[gridY*scaledGridWidth+gridX];
		if(gridValue > 0){
			currentScore += gridValue;
		}else{
			currentScore -= 1;
		}
	}
	return currentScore;
}

void localize(){
	ROS_INFO("---- Localizing");
	std::free(staticCloud);
	staticCloud = (geometry_msgs::Point32*)malloc(sizeof(geometry_msgs::Point32)*360);
	for(int i = 0; i < 360; ++i){
		staticCloud[i] = cloud.points[i];
	}	
	nav_msgs::Odometry staticOdom = odom;

	float gridResolution = occGrid.info.resolution;
	int gridWidth = occGrid.info.width;
	int gridHeight = occGrid.info.height;

	int lastKnownX = getGridX(odom.pose.pose.position.x, gridResolution);
	int lastKnownY = getGridY(odom.pose.pose.position.y, gridResolution);
	int lastKnownAngle = odom.pose.pose.orientation.z;

	float bestAngle = 0.0f;
	int bestX = 0;
	int bestY = 0;
	long bestValue = 0;

	int searchSize = 100;
	int searchScale = 10;
	int searchCells = searchSize/searchScale;
	int searchAngles = 20;

	int scaledGridWidth = gridWidth/searchScale;
	int scaledGridHeight = gridHeight/searchScale;

	int* scaledGrid = (int*)malloc(sizeof(int)*scaledGridWidth*scaledGridHeight);

	for(int j = 0; j < scaledGridHeight; ++j){
		for(int i = 0; i < scaledGridWidth; ++i){
			int gridVal = 0;
			for(int u = 0; u < searchScale; ++u){
				for(int v = 0; v < searchScale; ++v){
					int cellValue = getOccGridValue((i*scaledGridWidth)+v,(j*scaledGridHeight)+u);
					if(cellValue > gridVal){
						gridVal = cellValue;
					}
				}
			}
			scaledGrid[j*scaledGridWidth + i] = gridVal;
		}
	}

	float searchAngleDiff = 3.141593*2/searchAngles;
	for(int j = 0; j < searchCells; ++j){
		for(int i = 0; i < searchCells; ++i){
			for(float o = 0; o < 3.141593*2; o+=searchAngleDiff){
				long score = getConfigScore(j, i, o, scaledGrid, staticCloud, staticOdom, gridResolution*searchScale, searchScale, scaledGridWidth, scaledGridHeight);
				if(score > bestValue){
					bestValue = score;
					bestX = i;
					bestY = j;
					bestAngle = o;
				}
			}
		}
	}

	float tmpBestAngle = bestAngle;
	bestAngle = 0.0f;
	bestX = 0;
	bestY = 0;
	bestValue = 0;

	for(int j = 0; j < searchCells; ++j){
		for(int i = 0; i < searchCells; ++i){
			for(float o = -0.35; o < 0.35; o+=0.01){
				int x = bestX*scaledGridWidth + i;
				int y = bestY*scaledGridHeight + j;
				long score = getConfigScore(y, x, tmpBestAngle+o, occGrid, staticCloud, staticOdom, gridResolution, 1, gridWidth, gridHeight);
				if(score > bestValue){
					bestValue = score;
					bestX = i;
					bestY = j;
					bestAngle = tmpBestAngle+o;
				}
			}
		}
	}

	ROS_INFO("---- BestValue: %d, X: %d, Y: %d, Angle: %f", bestValue, bestX, bestY, bestAngle);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_localizer");


    ros::NodeHandle n;
	//Subscribe to Rosie's reported pose
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",100,odomCallback);
	//Subscribe to transformed LIDAR point cloud (Fixed to robot frame)
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::PointCloud>("/my_cloud",5,lidarCallback);
	//Subscribe to UPDATED Map
    ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1, gridCallback);

	load_time = ros::Time::now();
	ros::Rate loop_rate(1);

	while(ros::ok()){
		if(odomGotten && occGridGotten && lidarGotten){
			localize();
		}else{
			ROS_ERROR("Missing data for localization... %d %d %d", (int)odomGotten, (int)occGridGotten, (int)lidarGotten);
		}

	    ros::spinOnce();

    	loop_rate.sleep();

    }
}
