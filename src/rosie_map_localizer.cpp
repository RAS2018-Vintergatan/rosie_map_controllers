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
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>

#define PI 3.141592f
#define TWO_PI 6.28319f

boost::shared_ptr<sensor_msgs::PointCloud> lastPointCloud_ptr;
boost::shared_ptr<tf::TransformListener> laser_tfl_ptr;
boost::shared_ptr<tf::StampedTransform> laser_point_tf_ptr;

ros::Time load_time;

nav_msgs::OccupancyGrid occGrid;

nav_msgs::Odometry odom;

ros::Publisher pose_pub;
ros::Publisher localisationVisualisationPublisher;

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
	*lastPointCloud_ptr = msg;
	try{
		(*laser_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
		(*laser_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *laser_point_tf_ptr);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
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

/*
 * Returns the angle between -PI and +PI
 */
float capAngle(const float& angleIn){
	float angleOut = angleIn;
	while(angleOut < 0)
			angleOut += TWO_PI;
	while(angleOut > TWO_PI)
		angleOut -= TWO_PI;
	if(angleOut > PI){
		angleOut = (angleOut-TWO_PI);
	}
	return angleOut;
}

int getOccGridValue(int x, int y){
	int gridWidth = occGrid.info.width;
	int gridHeight = occGrid.info.height;
	return occGrid.data[y*gridWidth+x];
}

void publishCorrection(float correctionX, float correctionY, float correctionAngle){
	
	//ROS_INFO("Correction! X:%f, Y:%f, Yaw:%f",correctionX,correctionY,correctionAngle);

	geometry_msgs::PoseStamped correctionPose;

    correctionPose.header.stamp = ros::Time::now();
    correctionPose.header.frame_id = "world";

	correctionPose.pose.orientation.x = 0;
    correctionPose.pose.orientation.y = 0;
    correctionPose.pose.orientation.z = sin(correctionAngle);
    correctionPose.pose.orientation.w = cos(correctionAngle);

    correctionPose.pose.position.x = correctionX;
    correctionPose.pose.position.y = correctionY;
	correctionPose.pose.position.z = 0;

	pose_pub.publish(correctionPose);
}

char isPointInside(const int& x, const int& y, const int& gridWidth, const int& gridHeight){
	if(x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) 
		return 1;
	return 0;
}

tf::Vector3 getCorrectionFromPoint(tf::Vector3 point, tf::Vector3 centerPoint, nav_msgs::OccupancyGrid wallGrid){
	int gridWidth = wallGrid.info.width;
	int gridHeight = wallGrid.info.height;
	float gridResolution = wallGrid.info.resolution;

	tf::Vector3 pointFromCenterPoint(point.x()-centerPoint.x(),point.y()-centerPoint.y(),0);

	int centerGridX = getGridX(centerPoint.x(), gridResolution);
	int centerGridY = getGridY(centerPoint.y(), gridResolution);

	int pointGridX = getGridX(point.x(), gridResolution);
	int pointGridY = getGridY(point.y(), gridResolution);

	//Check if point is inside the grid, walls are the distance otherwise return the closest wall point
	if(!isPointInside(pointGridX, pointGridY, gridWidth, gridHeight)){
		tf::Vector3 returnVector(0,0,0);
		if(pointGridX < 0){
			returnVector.setX(-pointGridX*gridResolution);
		}else if(pointGridX >= gridWidth){
			returnVector.setX((gridWidth - pointGridX)*gridResolution);
		}
		if(pointGridY < 0){
			returnVector.setY(-pointGridY*gridResolution);
		}else if(pointGridY >= gridHeight){
			returnVector.setY((gridHeight - pointGridY)*gridResolution);
		}
		tf::Vector3 newPointFromCenterPoint(pointFromCenterPoint.x() + returnVector.x(),
											 pointFromCenterPoint.y() + returnVector.y(),
											 0);
		returnVector.setZ(capAngle(atan2(newPointFromCenterPoint.y(),newPointFromCenterPoint.x()) - atan2(pointFromCenterPoint.y(),pointFromCenterPoint.x())));
		return returnVector;
	}

	//Define search window size
	int smallestSearchDistance = -1;
	int searchSize = 0;
	while(smallestSearchDistance == -1){// && searchSize < ((gridHeight+gridWidth)>>1)){
		++searchSize;
		int x1 = pointGridX + searchSize;
		int y1 = pointGridY;

		int x2 = pointGridX - searchSize;
		int y2 = pointGridY;

		int x3 = pointGridX;
		int y3 = pointGridY + searchSize;

		int x4 = pointGridX;
		int y4 = pointGridY - searchSize;
		
		if(!isPointInside(x1, y1, gridWidth, gridHeight) ||
			!isPointInside(x2, y2, gridWidth, gridHeight) ||
			!isPointInside(x3, y3, gridWidth, gridHeight) ||
			!isPointInside(x4, y4, gridWidth, gridHeight)){
			// A point outside, smallest search window found
			smallestSearchDistance = searchSize;
		}else{
			if(wallGrid.data[y1*gridWidth + x1] == 125 ||
				wallGrid.data[y2*gridWidth + x2] == 125 ||
				wallGrid.data[y3*gridWidth + x3] == 125 ||
				wallGrid.data[y4*gridWidth + x4] == 125){
				// A point found a wall, smallest search window found
				smallestSearchDistance = searchSize;
			}
		}
	}
	
	//Search for absolute smallest distance
	float smallestDistance = -1;
	int smallestX = pointGridX;
	int smallestY = pointGridY;
	for(int j = -(smallestSearchDistance); j < (smallestSearchDistance); ++j){
		if(j < -(smallestSearchDistance)){
			continue;
		}else if(j > smallestSearchDistance){
			break;
		}
		for(int i = -(smallestSearchDistance); i < (smallestSearchDistance); ++i){
			if(i < -(smallestSearchDistance)){
				continue;
			}
			else if(i > smallestSearchDistance){
				break;
			}
			int sy = pointGridY+j;
			int sx = pointGridX+i;
			if(isPointInside(sx, sy, gridWidth, gridHeight)){
				if(wallGrid.data[sy*gridWidth+sx] == 125){
					float distance = sqrt(pow(i,2)+pow(j,2));
					if(smallestDistance < 0 || distance < smallestDistance){
						smallestDistance = distance;
						smallestX = sx;
						smallestY = sy;
						int absi = i < 0 ? -i:i;
						int absj = j < 0 ? -j:j;
						if(absi < absj){
							smallestSearchDistance = absi;
						}else{
							smallestSearchDistance = absj;
						}
					}
				}
			}
		}
	}

	//Now we have the point with the smallest distance
	tf::Vector3 returnVector((smallestX-pointGridX)*gridResolution, (smallestY-pointGridY)*gridResolution, 0);
	tf::Vector3 newPointFromCenterPoint(pointFromCenterPoint.x() + returnVector.x(),
										 pointFromCenterPoint.y() + returnVector.y(),
										 0);
	returnVector.setZ(capAngle(atan2(newPointFromCenterPoint.y(),newPointFromCenterPoint.x()) - atan2(pointFromCenterPoint.y(),pointFromCenterPoint.x())));
	
	return returnVector;
}

geometry_msgs::Point32 getMedianPointByDistance(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3){
	
	if(std::isnan(p1.x) || std::isnan(p1.y) ||
		std::isnan(p3.x) || std::isnan(p3.y)){
		return p2;
	}else if(std::isnan(p2.x) || std::isnan(p2.y)){
		return p1;
	}
	
	double dist1 = sqrt(p1.x*p1.x + p1.y*p1.y);
	double dist2 = sqrt(p2.x*p2.x + p2.y*p2.y);
	double dist3 = sqrt(p3.x*p3.x + p3.y*p3.y);
	
	if(dist1 < dist2 && dist2 < dist3){
		return p2;
	}else if(dist3 < dist2 && dist2 < dist1){
		return p2;
	}else if(dist2 < dist3 && dist3 < dist1){
		return p3;
	}else if(dist1 < dist3 && dist3 < dist2){
		return p3;
	}else{
		return p1;
	}
}

geometry_msgs::Point32 getCyclicPointCloudElement(int index, int max){
	while(index >= max){
		index -= max;
	}while(index < 0){
		index += max;
	}
	return lastPointCloud_ptr->points[index];
}

void localize(){

	visualization_msgs::Marker line_list;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.a = 1.0;
	line_list.color.b = 0.8;
	line_list.color.g = 0.4;
	line_list.scale.x = 0.01;
	line_list.header.stamp = ros::Time::now();
	line_list.header.frame_id = "world";

	double transformXSum = 0;
	double transformYSum = 0;
	double transformYawSum = 0;

	tf::Vector3 odomPoint(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);

	int numContributions = 0;
	for(int i = 0; i < 360; i+=1){
		geometry_msgs::Point32 pointInCloud = getMedianPointByDistance(
																		getCyclicPointCloudElement(i-1,360),
																		getCyclicPointCloudElement(i,360),
																		getCyclicPointCloudElement(i+1,360)
																	);
		
		if(std::isnan(pointInCloud.x) || std::isnan(pointInCloud.y) || std::isnan(pointInCloud.z)){
			continue;
		}

		tf::Vector3 point(pointInCloud.x,pointInCloud.y,pointInCloud.z);
		
		if(std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z())){
			continue;
		}

		tf::Vector3 point_tf = (*laser_point_tf_ptr) * point;

		tf::Vector3 diffVector = getCorrectionFromPoint(point_tf, odomPoint, occGrid);

		if(isnan(diffVector.z())){
			continue;
		}	
		
		if((diffVector.x() < -1.0f || diffVector.x() > 1.0f) && (diffVector.y() < -1.0f || diffVector.y() > 1.0f)){
			continue;
		}
			
		++numContributions;
		transformXSum += diffVector.x();
		transformYSum += diffVector.y();
		transformYawSum += diffVector.z();
		
		if(diffVector.x() > -0.2f && diffVector.x() < 0.2f && diffVector.y() > -0.2f && diffVector.y() < 0.2f){
			numContributions += 3;
			transformXSum += diffVector.x()*3;
			transformYSum += diffVector.y()*3;
			transformYawSum += diffVector.z()*3;
		}
		if(sqrt(point.x()*point.x() + point.y()*point.y()) < 0.5){
			++numContributions;
			transformXSum += diffVector.x();
			transformYSum += diffVector.y();
			transformYawSum += diffVector.z();
		}
		
		geometry_msgs::Point p;
		p.x = point_tf.x();
		p.y = point_tf.y();
		line_list.points.push_back(p);
		p.x += diffVector.x();
		p.y += diffVector.y();
		line_list.points.push_back(p);
		//ROS_INFO("Diffvector! X:%f, Y:%f, Yaw:%f",diffVector.x(),diffVector.y(),diffVector.z());
	}
	//ROS_INFO("Correction Sum! X:%f, Y:%f, Yaw:%f",transformXSum,transformYSum,transformYawSum);
	publishCorrection(transformXSum/numContributions,transformYSum/numContributions,transformYawSum/numContributions);
	
	localisationVisualisationPublisher.publish(line_list);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_localizer");

	
	laser_tfl_ptr.reset(new tf::TransformListener);
	laser_point_tf_ptr.reset(new tf::StampedTransform);
	lastPointCloud_ptr.reset(new sensor_msgs::PointCloud);

    ros::NodeHandle n;
	//Subscribe to Rosie's reported pose
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",100,odomCallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_correction",1);
	localisationVisualisationPublisher = n.advertise<visualization_msgs::Marker>("/localisation_lines",1);
	//Subscribe to transformed LIDAR point cloud (Fixed to robot frame)
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::PointCloud>("/my_cloud",5,lidarCallback);
	//Subscribe to UPDATED Map
    ros::Subscriber grid_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1, gridCallback);

	load_time = ros::Time::now();
	ros::Rate loop_rate(50);

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
