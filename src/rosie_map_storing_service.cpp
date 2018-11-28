#include "ros/ros.h"
#include <rosie_map_controller/MapStoring.h>

#include <fstream>

#include <cstdlib>

string wallFilePath;
string objectFilePath;
string batteryFilePath;

bool requestRerunCallback(rosie_map_controller::RequestStoring::Request &req, rosie_map_controller::RequestStoring::Response &res){
	std::ofstream ofs;
	
	rosie_map_controller::WallDefinition[]& walls = req.NewWalls;
	ofs.open(wallFilePath, std::ofstream::out);
	for(int i = 0; i < walls.size(); ++i){
		ofs << walls[i].x1 << " " << walls[i].y1 << " ";
		ofs << walls[i].x2 << " " << walls[i].y2 << " ";
		ofs << walls[i].certainty << std::endl;
	}
	ofs.close();
	
	rosie_map_controller::ObjectPosition[]& objects = req.Objects;
	ofs.open(objectFilePath, std::ofstream::out);
	for(int i = 0; i < objects.size(); ++i){
		ofs << objects[i].id << " ";
		ofs << objects[i].x << " " << objects[i].y << " ";
		ofs << objects[i].value << " ";
		ofs << objects[i].name << std::endl;
	}
	ofs.close();
	
	rosie_map_controller::BatteryPosition[]& batteries = req.Batteries;
	ofs.open(batteryFilePath, std::ofstream::out);
	for(int i = 0; i < batteries.size(); ++i){
		ofs << objects[i].x << " " << objects[i].y << " ";
		ofs << objects[i].certainty << std::endl;
	}
	ofs.close();
	
	return true;
}

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "rosie_rrt_server");

  ros::NodeHandle n;

  n.param<string>("wall_save_file", wallFilePath, "wall_save_file.txt");
  n.param<string>("object_save_file", objectFilePath, "obj_save_file.txt");
  n.param<string>("battery_save_file", batteryFilePath, "battery_save_file.txt");
  
  ros::ServiceServer storeService = n.advertiseService<rosie_map_controller::RequestStoring::Request, rosie_map_controller::RequestStoring::Response>("request_rerun", requestStoringCallback);
  ros::spin();

  return 0;
}