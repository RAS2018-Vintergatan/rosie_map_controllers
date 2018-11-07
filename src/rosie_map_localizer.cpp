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
#include <tf/transform_broadcaster.h>
#include <iostream>

sensor_msgs::PointCloud cloud;
geometry_msgs::Point32* staticCloud;

ros::Time load_time;

nav_msgs::OccupancyGrid occGrid;

nav_msgs::Odometry odom;

ros::Publisher pose_pub;

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

void publishCorrection(float newX, float newY, float newAngle){
	
	geometry_msgs::PoseStamped newPose;

    newPose.header.stamp = ros::Time::now();;
    newPose.header.frame_id = "world";

	newPose.pose.orientation.x = 0;
    newPose.pose.orientation.y = 0;
    newPose.pose.orientation.z = sin(newAngle/2);
    newPose.pose.orientation.w = cos(newAngle/2);

    newPose.pose.position.x = newX;
    newPose.pose.position.y = newY;
	newPose.pose.position.z = 0;

	pose_pub.publish(newPose);
}


int getConfigScore(int conf_grid_x, int conf_grid_y, float conf_angle, float robot_angle, int* grid, geometry_msgs::Point32* cloud, nav_msgs::Odometry odom,
					 float gridResolution, int scaledGridWidth, int scaledGridHeight){
	int robotDiffX = (conf_grid_x/gridResolution)-odom.pose.pose.position.x;
	int robotDiffY = (conf_grid_y/gridResolution)-odom.pose.pose.position.y;

	//publishCorrection(conf_grid_x*gridResolution,conf_grid_y*gridResolution,conf_angle);

	/*float closestDist = -1;
	int closestAngle = 0;*/

	//ROS_INFO("on %d:%d, scaledGridWidth: %d, scaledGrid:%d", conf_grid_x,conf_grid_y,scaledGridWidth,grid[conf_grid_y*scaledGridWidth+conf_grid_x]);


	int amountOkay = 0;
	int amountBad = 0;

	int staticScore = -sqrt(std::pow(robotDiffX,2)+std::pow(robotDiffY,2)) - std::abs(conf_angle-robot_angle)*10;
	int currentScore = 0;
	for(int i = 0; i < 360; ++i){
		geometry_msgs::Point32 point = cloud[i];				
	
		float distanceFromRobot = sqrt(cloud[i].x*cloud[i].x + cloud[i].y*cloud[i].y);
		if(distanceFromRobot > 0.8)
			continue;		

		geometry_msgs::Point32 transformedPoint;
		transformedPoint.x = conf_grid_x*gridResolution + point.x*cos(conf_angle) + point.y*sin(conf_angle);
		transformedPoint.y = conf_grid_y*gridResolution + point.y*cos(conf_angle) - point.x*sin(conf_angle);

		//float dist = sqrt((transformedPoint.x*transformedPoint.x)+(transformedPoint.y*transformedPoint.y));
		//if(dist > 1.5){
		//	continue;
		//}

		int gridX = getGridX(transformedPoint.x, gridResolution);
		int gridY = getGridY(transformedPoint.y, gridResolution);

		/*if(closestDist < 0 || dist < closestDist){
			closestDist = dist;
			closestAngle = i;
		}*/

		if(gridX >= scaledGridWidth || gridX < 0){
			if(gridX <= scaledGridWidth + 0.5/gridResolution && gridX > 0 - 0.5/gridResolution){
				currentScore+=12;
				++amountOkay;
			}
			continue;
		}
		if(gridY >= scaledGridHeight || gridY < 0){
			if(gridY <= scaledGridHeight + 0.5/gridResolution && gridY > 0 - 0.5/gridResolution){
				currentScore+=12;
				++amountOkay;
			}
			continue;
		}

		int gridValue = grid[gridY*scaledGridWidth+gridX];
		if(gridValue > 0){
			++amountOkay;
			if(gridValue == 125){
				currentScore+= 1000;
			}
			//currentScore += gridValue;
		}else{
			currentScore -= 1;
			++amountBad;
		}
	}
	/*if(angle == 0){
		geometry_msgs::Point32 transformedPoint;
		transformedPoint.x = cloud[closestAngle].x*cos(angle) + cloud[closestAngle].y*sin(angle);
		transformedPoint.y = -cloud[closestAngle].x*sin(angle) + cloud[closestAngle].y*cos(angle);
		ROS_INFO("Closest angle: %d @ %f, \tx:%f|y:%f", closestAngle, closestDist,transformedPoint.x,transformedPoint.y);
	}*/
	return (currentScore)*(float(amountOkay/(amountBad+amountOkay))) + staticScore*(float(amountBad/(amountBad+amountOkay)));
}

int getConfigScore(int conf_grid_x, int conf_grid_y, float conf_angle, geometry_msgs::Point32* cloud, nav_msgs::Odometry odom,
					 float gridResolution, int gridWidth, int gridHeight){
	int robotDiffX = (conf_grid_x/gridResolution)-odom.pose.pose.position.x;
	int robotDiffY = (conf_grid_y/gridResolution)-odom.pose.pose.position.y;

	//publishCorrection(conf_grid_x*gridResolution,conf_grid_y*gridResolution,conf_angle);
	
	//ROS_INFO("accurateGrid:%d", getOccGridValue(conf_grid_x,conf_grid_y));

	/*float closestDist = -1;
	int closestAngle = 0;*/

	//int staticScore = -sqrt(std::pow(robotDiffX,2)+std::pow(robotDiffY,2)) - std::abs(conf_angle)*100;
	int currentScore = 0;
	for(int i = 0; i < 360; ++i){
		geometry_msgs::Point32 point = cloud[i];

		geometry_msgs::Point32 transformedPoint;
		transformedPoint.x = conf_grid_x*gridResolution + point.x*cos(conf_angle) + point.y*sin(conf_angle);
		transformedPoint.y = conf_grid_y*gridResolution + point.y*cos(conf_angle) - point.x*sin(conf_angle);

		//float dist = sqrt((transformedPoint.x*transformedPoint.x)+(transformedPoint.y*transformedPoint.y));
		//if(dist > 1.5){
		//	continue;
		//}

		int gridX = conf_grid_x+getGridX(transformedPoint.x, gridResolution);
		int gridY = conf_grid_y+getGridY(transformedPoint.y, gridResolution);

		if(gridX >= gridWidth || gridX < 0){
			if(gridX <= gridWidth + 0.5/gridResolution && gridX > 0 - 0.5/gridResolution){
				currentScore+=12;
			}
			continue;
		}
		if(gridY >= gridHeight || gridY < 0){
			if(gridY <= gridHeight + 0.5/gridResolution && gridY > 0 - 0.5/gridResolution){
				currentScore+=12;
			}
			continue;
		}

		int gridValue = getOccGridValue(gridX,gridY);
		if(gridValue > 0){
			if(gridValue == 125){
				currentScore+= 400;
			}
			//currentScore += gridValue;
		}else{
			
			//currentScore -= 10;
		}
	}
	return (currentScore);//+ staticScore*100;
}

void localize(){

	/* Initialize variables */
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
	double lastKnownAngle = 0;

	tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
					 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, lastKnownAngle);

	/* Generate scaled version of the map */

	int searchSize = 20;
	int searchScale = 5;
	int searchCells = searchSize/searchScale;
	int searchCellsHalf = searchCells>>1;
	int searchAngles = 36;

	int scaledGridWidth = gridWidth/searchScale;
	int scaledGridHeight = gridHeight/searchScale;

	int* scaledGrid = (int*)malloc(sizeof(int)*scaledGridWidth*scaledGridHeight);

	for(int j = 0; j < scaledGridHeight; ++j){
		for(int i = 0; i < scaledGridWidth; ++i){
			int gridVal = 0;
			for(int u = 0; (j*searchScale)+u < gridHeight && u < searchScale; ++u){
				for(int v = 0; (i*searchScale)+v < gridWidth && v < searchScale; ++v){
					int cellValue = getOccGridValue((i*searchScale)+v,(j*searchScale)+u);
					if(cellValue > gridVal){
						gridVal = cellValue;
					}if(cellValue != 0 && cellValue != 120){
						ROS_INFO("CellValue on |x:%d|y:%d|:%d",(i*searchScale)+v,(j*searchScale)+u,cellValue);
					}
				}
			}
			scaledGrid[j*scaledGridWidth + i] = gridVal;
		}
	}

	/* Search in a scaled version of the map */

	float bestAngle = 0.0f;
	int bestX = -1;
	int bestY = -1;
	int bestValue = -1;

	float searchAngleDiff = 3.141593*2/searchAngles;
	for(int j = -searchCells; j < searchCells; ++j){
		int y = (lastKnownY/searchScale) + j;
		if(y < 0 || y > scaledGridHeight){
			continue;
		}
		for(int i = -searchCells; i < searchCells; ++i){
			int x = (lastKnownX/searchScale) + i;
			if(x < 0 || x > scaledGridWidth){
				continue;
			}
			for(float o = 0; o < 3.141593*2; o+=searchAngleDiff){
				int score = getConfigScore(x, y, o, lastKnownAngle, scaledGrid, staticCloud, staticOdom, gridResolution*searchScale, scaledGridWidth, scaledGridHeight);
				if(score > bestValue || bestX == -1 || bestY == -1){
					bestValue = score;
					bestX = x;
					bestY = y;
					bestAngle = o+bestAngle;
				}
			}
		}
	}

	if(0&&bestValue < 0){
		ROS_ERROR("Could not find the position accurately enough!");
		std::free(scaledGrid);
		return;
	}

	/* Search more accurately around the found scaled config */	

	/*bestAngle = 3.1415;
	bestX = 20;
	bestY = 20;*/

	float bestAngleAccurate = 0.0f;
	int bestXAccurate = -1;
	int bestYAccurate = -1;
	int bestValueAccurate = -1;

	for(int j = -searchCells; j < searchCells; ++j){
		int y = bestY*searchScale + j;
		if(y < 0 || y > gridHeight){
			continue;
		}
		for(int i = -searchCells*2; i < searchCells*2; ++i){
			for(float o = -searchAngleDiff; o < searchAngleDiff; o+=0.01){	
				int x = bestX*searchScale + i;
				if(x < 0 || x > gridWidth){
					continue;
				}
				int score = getConfigScore(x, y, bestAngle+o, staticCloud, staticOdom, gridResolution, gridWidth, gridHeight);
				if(score > bestValueAccurate || bestXAccurate == -1 || bestYAccurate == -1){
					bestValueAccurate = score;
					bestXAccurate = x;
					bestYAccurate = y;
					bestAngleAccurate = bestAngle+o;
				}
			}
		}
	}
	if(0&&bestValueAccurate < 0){
		ROS_ERROR("Could not find the position accurately enough!");
		std::free(scaledGrid);
		return;
	}
	std::free(scaledGrid);
	ros::Rate loop_rate(1);
	ROS_INFO("---- World GRID ---- width: %d, height: %d, resolution: %f",gridWidth,gridHeight,gridResolution);
	ROS_INFO("---- Scaled GRID ---- width: %d, height: %d",scaledGridWidth,scaledGridHeight);
	ROS_INFO("---- LAST ---- xWorld: %f, xGrid: %d, yWorld: %f, yGrid: %d", odom.pose.pose.position.x, lastKnownX, odom.pose.pose.position.y, lastKnownY);
	ROS_INFO("---- Scaled ---- best value: %d, X: %d, Y: %d, Angle: %f", bestValue, bestX, bestY, bestAngle);
	ROS_INFO("---- Accurate ---- BestValue: %d, X: %d, Y: %d, Angle: %f", bestValueAccurate, bestXAccurate, bestYAccurate, bestAngleAccurate);

	ROS_INFO("---- Publishing");
	
	publishCorrection(bestXAccurate*gridResolution,bestYAccurate*gridResolution,bestAngleAccurate);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_localizer");


    ros::NodeHandle n;
	//Subscribe to Rosie's reported pose
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",100,odomCallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_correction",1);
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
