#include "ros/ros.h"
#include <rosie_map_controller/BatteryPosition.h>
#include <rosie_map_controller/ObjectPosition.h>
#include <rosie_map_controller/WallDefinition.h>
#include <rosie_map_controller/MapStoring.h>
#include <rosie_map_controller/RequestStoring.h>
#include <string>
#include <fstream>

#include <cstdlib>

std::string wallFilePath;
std::string objectFilePath;
std::string batteryFilePath;

bool requestStoringCallback(rosie_map_controller::RequestStoring::Request &req, rosie_map_controller::RequestStoring::Response &res){
	std::ofstream ofs;
	
 	std::vector<rosie_map_controller::WallDefinition> walls = req.send.NewWalls;
	ofs.open(wallFilePath.c_str());
	for(int i = 0; i < walls.size(); ++i){
		ofs << walls[i].x1 << " " << walls[i].y1 << " ";
		ofs << walls[i].x2 << " " << walls[i].y2 << " ";
		ofs << walls[i].certainty << std::endl;
	}
	ofs.close();
	
	std::vector<rosie_map_controller::ObjectPosition> objects = req.send.Objects;
	ofs.open(objectFilePath.c_str());
	for(int i = 0; i < objects.size(); ++i){
		ofs << objects[i].id << " ";
		ofs << objects[i].x << " " << objects[i].y << " ";
		ofs << objects[i].value << " ";
		ofs << objects[i].name << std::endl;
	}
	ofs.close();
	
	std::vector<rosie_map_controller::BatteryPosition> batteries = req.send.Batteries;
	ofs.open(batteryFilePath.c_str());
	for(int i = 0; i < batteries.size(); ++i){
		ofs << batteries[i].x << " " << batteries[i].y << " ";
		ofs << batteries[i].certainty << std::endl;
	}
	ofs.close();
	
	return true;
}

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "rosie_map_storing_service");

  ros::NodeHandle n;

  n.param<std::string>("wall_save_file", wallFilePath, "wall_save_file.txt");
  n.param<std::string>("object_save_file", objectFilePath, "obj_save_file.txt");
  n.param<std::string>("battery_save_file", batteryFilePath, "battery_save_file.txt");
  
  ros::ServiceServer storeService = n.advertiseService<rosie_map_controller::RequestStoring::Request, rosie_map_controller::RequestStoring::Response>("request_store_map", requestStoringCallback);
  ros::spin();

  return 0;
}
