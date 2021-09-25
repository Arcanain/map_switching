#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/SetMap.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include "Vector.h"

using namespace std;

const int select_maps = 6;

vector<nav_msgs::OccupancyGrid> map_array(select_maps);

std_msgs::Int32 map_num;

void map_callback0(const nav_msgs::OccupancyGrid& map) {
    map_array.at(0) = map;
}
void map_callback1(const nav_msgs::OccupancyGrid& map) {
    map_array.at(1) = map;
}
void map_callback2(const nav_msgs::OccupancyGrid& map) {
    map_array.at(2) = map;
}
void map_callback3(const nav_msgs::OccupancyGrid& map) {
    map_array.at(3) = map;
}
void map_callback4(const nav_msgs::OccupancyGrid& map) {
    map_array.at(4) = map;
}
void map_callback5(const nav_msgs::OccupancyGrid& map) {
    map_array.at(5) = map;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simple_map_switch");

    ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);

    ros::NodeHandle lSubscriber("");

    ros::Subscriber map_sub0 = lSubscriber.subscribe("/map0", 50, map_callback0);
    ros::Subscriber map_sub1 = lSubscriber.subscribe("/map1", 50, map_callback1);
    ros::Subscriber map_sub2 = lSubscriber.subscribe("/map2", 50, map_callback2);
    ros::Subscriber map_sub3 = lSubscriber.subscribe("/map3", 50, map_callback3);
    ros::Subscriber map_sub4 = lSubscriber.subscribe("/map4", 50, map_callback4);
    ros::Subscriber map_sub5 = lSubscriber.subscribe("/map5", 50, map_callback5);

    map_num.data = 0;
    int count = 0;
    ros::Rate loop_rate(50);

    while (n.ok()) {
        
        nav_msgs::SetMap set_map;
        
        if (count >= 0 && count < 100) {
            map_num.data = 0;
        } else if (count >= 100 && count < 200) {
            map_num.data = 1;
        } else if (count >= 200 && count < 300) {
            map_num.data = 2;
        } else if (count >= 300 && count < 400) {
            map_num.data = 3;
        } else if (count >= 400 && count < 500) {
            map_num.data = 4;
        } else if (count >= 500 && count < 600) {
            map_num.data = 5;
        } else if (count >= 600) {
            map_num.data = 0;
        }

        map_pub.publish(map_array.at(map_num.data));
        ros::spinOnce();
        loop_rate.sleep();

        count++;
    }
}