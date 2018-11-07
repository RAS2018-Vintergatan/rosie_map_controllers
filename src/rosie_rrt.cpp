#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
int EPS = 100;
int numNodes = 2000;
int offset[] = {0, 0};

struct node{
    float coord[2];
    double cost = 0;
    int parent = 0;
} q_start, q_goal;


float pi = 3.14159265359;
float resolution;
std_msgs::Int16 width = 1;
std_msgs::Int16 height = 1;
int nWalls = 7;
float robotsize = 0.2;
std_msgs::UInt8 *occGrid;
//visualization_msgs::Marker *markers;
ros::Time load_time;

nav_msgs::Odometry pose;
nav_msgs::OccupancyGrid mapgrid;
std_msgs::Int16 *walls;

void objectsCallback(visualization_msgs::MarkerArray){ //get object positions (sorted after value and distance)

}

void wallCallback(std_msgs::Int16MultiArray msg){ //should contain width, height and all wall points = k*4 + 2 elements
    float size = sizeof(msg)*sizeof(msg[0].data);
    walls = (std_msgs::Int16*)malloc(size-2);
    memcpy(walls, msg, sizeof(*walls)*(size-2));
    width = msg[0].data;
    height = msg[1].data;
}

void poseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
    q_start.coord = {pose.pose.pose.position.y + offset[0] ,pose.pose.pose.position.x + offset[1]};
}

bool ccw(std::vector<float> A,std::vector<float> B,std::vector<float> C){
     bool val = (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0]);
     return val;
}

float dist(std::vector<float> q1, std::vector<float> q2){
    float d = sqrt((q1[0]-q2[0])^2 + (q1[1]-q2[1])^2);
    return d;
}

std::vector<float> steer(std::vector<float> qr, std::vector<float> qn, float val, float eps){
    std::vector<float>  qnew = { 0, 0};

    //Steer twoards qn with maximum step size of eps
    if(val >= eps){
        qnew[0] = qn[0] + ((qr[0]-qn[0])*eps)/dist(qr, qn);
        qnew[1] = qn[1] + ((qr[1]-qn[1])*eps)/dist(qr, qn);
    }else{
        qnew[0] = qr[0];
        qnew[1] = qr[1];
    }
    std::vector<float> A = {qnew[0], qnew[1]};
    return A;
}

bool noCollision(std::vector<float> n2, std::vector<float> n1, float walls[nwalls][4]){         //array definition might be wrong
    std::vector<float> A = n1;
    std::vector<float> B = n2;

    bool nc = true;
    for(int i =0 ; i<nWalls; ++i){
        std::vector<float> wall = walls[i][];
        float dist = sqrt((wall[2]-wall[0])^2+(wall[3]-wall[1])^2);
        float m1 = (wall[2]-wall[0])/dist;
        float m2 = (wall[3]-wall[1])/dist;
        int r = 20;

		float x1,y1,x2,y2;
        if(m1 >= 0){
            x1 = wall[0]-m1*r;
            y1 = wall[1]-m2*r;
            x2 = wall[2]+m1*r;
            y2 = wall[3]+m2*r;
        }else{
            x1 = wall[0]+m1*r;
            y1 = wall[1]+m2*r;
            x2 = wall[2]-m1*r;
            y2 = wall[3]-m2*r;
        }

        float n1 = 1-m1;
        float n2 = 1-m2;

        std::vector<float> P1(1,0);
		P1[0] = x1-n1*r;
		P1[1] = y1+n2*r;
        std::vector<float> P2(1,0);
		P2 = {x1+n1*r , y1-n2*r};
        std::vector<float> P3(1,0);
		P3 = {x2-n1*r , y2+n2*r};
        std::vector<float> P4(1,0);
		P4 = {x2+n1*r , y2-n2*r};

        std::vector<float> C1(1,0);
		C1 = P1;
        std::vector<float> C2(1,0);
		C2 = P1;
        std::vector<float> C3(1,0);
		C3 = P3;
        std::vector<float> C4(1,0);
		C4 = P3;
        std::vector<float> D1(1,0);
		D1 = P4;
        std::vector<float> D2(1,0); 
		D2 = P2;
        std::vector<float> D3(1,0);
		D3 = P2;
        std::vector<float> D4(1,0);
		D4 = P4;

        bool ints1 = ccw(A,C1,D1) != ccw(B,C1,D1) && ccw(A,B,C1) != ccw(A,B,D1);
        bool ints2 = ccw(A,C2,D2) != ccw(B,C2,D2) && ccw(A,B,C2) != ccw(A,B,D2);
        bool ints3 = ccw(A,C3,D3) != ccw(B,C3,D3) && ccw(A,B,C3) != ccw(A,B,D3);
        bool ints4 = ccw(A,C4,D4) != ccw(B,C4,D4) && ccw(A,B,C4) != ccw(A,B,D4);

        if(ints1==0 && ints2==0 && ints3==0 && ints4==0 &&nc ==1){
            nc = true;
        }else{
            nc = false;
        }
    return nc;
    }

}

int seq = 0;
void publishGrid(){
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

    //mapgrid.data = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*width*height);
    for(int i = 0; i < width*height; ++i){
        mapgrid.data.push_back(0);                              // put values in with path
    }

    grid_publisher.publish(mapgrid);
}

float randZO()
{
    return rand() / (RAND_MAX + 1.);
}

void runRRT(){
    node nodes[numNodes];
    nodes[0] = q_start;
    bool break_status = false;


    for(int i=0; i<numNodes; ++i){
        std::vector<float> q_rand(1,0);
		q_rand = { (width*randZO()), (height*randZO())};

        for(int j=0; j<=i; ++j){
            int r=50;
            std::vector<float> d(1,0);
			d = nodes[j].coord - q_goal.coord;
            if(sqrt(d[0]^2+d[1]^2) <= r){
                break_status = true;
            }
        }
        if(break_status == true){
            break;
        }

        float ndist[i] = {};
        for(int j=0;j<=i;++j){
            node n = nodes[j];
            float tmp = dist(n.coord, q_rand);
            ndist[j] = tmp;
        }
        int it = std::min_element(std::begin(ndist), std::end(ndist));
        std::size_t index = std::distance(std::begin(playerSums), it);
        int min = *it;

        node q_near = nodes(it);
        node q_new;
        q_new.coord = steer(q_rand, q_near.coord, val, EPS);

        if (noCollision(q_rand, q_near.coord, obstacle)){

            q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;

            // Within a radius of r, find all existing nodes
            node q_nearest[i];
            int r = 50;
            int neighbor_count = 1;
            for ( int j = 0; j<=i; ++j){
                if( noCollision(nodes[j].coord, q_new.coord, obstacle) && dist(nodes[j].coord, q_new.coord) <= r){
                    q_nearest[neighbor_count].coord = nodes[j].coord;
                    q_nearest[neighbor_count].cost = nodes[j].cost;
                    neighbor_count = neighbor_count+1;
                }
            }

            // Initialize cost to currently known value
            node q_min = q_near;
            double C_min = q_new.cost;

            // Iterate through all nearest neighbors to find alternate lower
            // cost paths

            for( int k = 0; k<=i; k++){
                if (noCollision(&q_nearest[k].coord, &q_new.coord, obstacle) && &q_nearest[k].cost + dist(&q_nearest[k].coord, &q_new.coord) < C_min){
                    q_min = q_nearest[k];
                    C_min = &q_nearest[k].cost + dist(q_nearest[k].coord, q_new.coord);

                }
            }

            // Update parent to least cost-from node
            for (int j=0; j<=i; ++j){
                if (nodes[j].coord == q_min.coord){
                    q_new.parent = j;
                }
            }

            // Append to nodes
            nodes.push_back(q_new);

        }

    }

}


int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_provider");

    ros::NodeHandle n;
    ros::Subscriber obj_sub = n.subscribe<visualization_msgs::MarkerArray>("/visualization_marker", 1, objectsCallback);
    ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, poseCallback);
    ros::Subscriber wall_sub = n.subscribe<std_msgs::UInt16>("/walls", 1, wallCallback);
    ros::Publisher grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_path_grid",1);

    ros::Rate loop_rate(10);
 
	load_time = ros::Time::now();

    while(ros::ok()){
        //check initialization
        // run RRT*
        runRRT();
        // publishGrid();
        //

        // modes getTarget, explore, home, re-run after map-update

       //update path when obstacle detected


	    ros::spinOnce();
    	loop_rate.sleep();
    }
}
