#include "ros/ros.h"
#include <rosie_map_controller/BatteryPosition.h>
#include <rosie_map_controller/ObjectPosition.h>
#include <rosie_map_controller/WallDefinition.h>
#include <rosie_map_controller/MapStoring.h>
#include <rosie_map_controller/RequestStoring.h>
#include <rosie_map_controller/RequestLoading.h>
#include <string>
#include <iostream>
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

bool requestLoadingCallback(rosie_map_controller::RequestLoading::Request &req, rosie_map_controller::RequestLoading::Response &res){

	std::ifstream map_fs;

	map_fs.open(wallFilePath.c_str());
	std::string line;
	while(getline(map_fs, line)){
		double x1 = -1, x2 = -1,
				y1 = -1, y2 = -1;
		int certainty = 0;
		
		std::istringstream line_stream(line);
		
		line_stream >> x1 >> y1 >> x2 >> y2 >> certainty;

		rosie_map_controller::WallDefinition wall;

		wall.x1 = x1;
		wall.y1 = y1;
		wall.x2 = x2;
		wall.y2 = y2;
		wall.certainty = certainty;
		res.mappings.NewWalls.push_back(wall);
	}

	//std::string line;
	map_fs.open(objectFilePath.c_str());
	while(getline(map_fs, line)){
		int id;
		double x = -1, y = -1;
		int value = 0;
		std::string name;
		
		std::istringstream line_stream(line);
		
		line_stream >> id >> x >> y >> value >> name;

		rosie_map_controller::ObjectPosition object;

		object.id = id;
		object.x = x;
		object.y = y;
		object.value = value;
		object.name = name;
		res.mappings.Objects.push_back(object);
	}

	map_fs.open(batteryFilePath.c_str());
	//std::string line;
	while(getline(map_fs, line)){
		double x = -1, y = -1;
		int certainty = 0;
		
		std::istringstream line_stream(line);
		
		line_stream >> x >> y >> certainty;

		rosie_map_controller::BatteryPosition battery;

		battery.x = x;
		battery.y = y;
		battery.certainty = certainty;
		res.mappings.Batteries.push_back(battery);
	}

	return true;
}

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "rosie_map_storing_service");

  ros::NodeHandle n;

  n.param<std::string>("wall_save_file", wallFilePath, "wall_save_file.txt");
  n.param<std::string>("object_save_file", objectFilePath, "obj_save_file.txt");
  n.param<std::string>("battery_save_file", batteryFilePath, "battery_save_file.txt");
  
  //Storing:
  ros::ServiceServer storeService = n.advertiseService<rosie_map_controller::RequestStoring::Request, rosie_map_controller::RequestStoring::Response>("request_store_mapping", requestStoringCallback);
  //Loading:
  ros::ServiceServer loadService = n.advertiseService<rosie_map_controller::RequestLoading::Request, rosie_map_controller::RequestLoading::Response>("request_load_mapping", requestLoadingCallback);

  ros::spin();

  return 0;
}
