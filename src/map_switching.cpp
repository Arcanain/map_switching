#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/SetMap.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include "Vector.h"
#include "tf_lis.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define DEBUG_ON

using namespace std;

const int select_maps = 6;

vector<nav_msgs::OccupancyGrid> map_array(select_maps);

std_msgs::Int32 map_num;

geometry_msgs::Pose current_robot_pose;

// 各種map切り替え時のx, y座標
float switching_waypoint_x;
float switching_waypoint_y;
float switching_waypoint_w;
bool map_chenge_flag[6] = {true, false, false, false, false, false};

nav_msgs::OccupancyGrid map_reconstruction(nav_msgs::OccupancyGrid map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;
    
    map.info.origin.position.x = current_robot_pose.position.x;
    map.info.origin.position.y = current_robot_pose.position.y;
    map.info.origin.position.z = current_robot_pose.position.z;
    map.info.origin.orientation.x = current_robot_pose.orientation.x;
    map.info.origin.orientation.y = current_robot_pose.orientation.y;
    map.info.origin.orientation.z = current_robot_pose.orientation.z;
    map.info.origin.orientation.w = current_robot_pose.orientation.w;
    
    /*
    map.info.origin.position.x = current_robot_pose.position.x;
    map.info.origin.position.y = current_robot_pose.position.y;
    map.info.origin.position.z = current_robot_pose.position.z;
    map.info.origin.orientation.x = current_robot_pose.orientation.x;
    map.info.origin.orientation.y = current_robot_pose.orientation.y;
    map.info.origin.orientation.z = -0.5;
    map.info.origin.orientation.w = 0.1;
    */
    map.data = map_msg.data;

    return map;
}

// map0
void map_callback0(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(0) = map_msg;
}

// map1
void map_callback1(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(1) = map_reconstruction(map_msg);
}

//map2
void map_callback2(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(2) = map_reconstruction(map_msg);
}

//map3
void map_callback3(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(3) = map_reconstruction(map_msg);
}

//map4
void map_callback4(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(4) = map_reconstruction(map_msg);
}

//map5
void map_callback5(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(5) = map_reconstruction(map_msg);
}

double geometry_quat_to_yaw(geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    double roll,pitch,yaw;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    return yaw;
}

Vector Pose_to_vec(geometry_msgs::Pose pose){
    Vector ret_vec;
    ret_vec.x   = pose.position.x;
    ret_vec.y   = pose.position.y;
    ret_vec.yaw = geometry_quat_to_yaw(pose.orientation);
    return ret_vec;
}

geometry_msgs::PoseWithCovarianceStamped vec_to_PoseWithCovarianceStamped(Vector pos){
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    initial_pose.pose.pose.position.x = pos.x;
    initial_pose.pose.pose.position.y = pos.y;
    initial_pose.pose.pose.orientation.z = pos.get_qz();
    initial_pose.pose.pose.orientation.w = pos.get_qw();
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "map";
    initial_pose.pose.covariance[0]=0.05;//0.25;
    initial_pose.pose.covariance[7]=0.05;//0.25;
    initial_pose.pose.covariance[35]=0.01;//0.06853891945200942;
    return initial_pose;
}

void map_to_baselink_callback(const geometry_msgs::Pose &msg) {
    /*
    current_robot_pose.position.x = msg.position.x;
    current_robot_pose.position.y = msg.position.y;
    current_robot_pose.position.z = msg.position.z;
    current_robot_pose.orientation.x = msg.orientation.x;
    current_robot_pose.orientation.y = msg.orientation.y;
    current_robot_pose.orientation.z = msg.orientation.z;
    current_robot_pose.orientation.w = msg.orientation.w;
     */
    
    current_robot_pose.position.x = msg.position.x;
    current_robot_pose.position.y = msg.position.y;
    current_robot_pose.position.z = msg.position.z;
    current_robot_pose.orientation.x = msg.orientation.x;
    current_robot_pose.orientation.y = msg.orientation.y;
    current_robot_pose.orientation.z = msg.orientation.z;
    current_robot_pose.orientation.w = msg.orientation.w;
}

void waypoint_callback(const visualization_msgs::Marker &msg) {
    switching_waypoint_x = msg.pose.position.x;
    switching_waypoint_y = msg.pose.position.y;
    switching_waypoint_w = msg.pose.orientation.w;

    if (switching_waypoint_x == -24.4127 && switching_waypoint_y == -28.9142 && switching_waypoint_y == -0.999601) {
        map_chenge_flag[0] = false;
        map_chenge_flag[1] = true;
    } else if (switching_waypoint_x == 203.174 && switching_waypoint_y == -82.518 && switching_waypoint_y == -0.721927) {
        map_chenge_flag[1] = false;
        map_chenge_flag[2] = true;
    } else if (switching_waypoint_x == 5.92503 && switching_waypoint_y == 148.57 && switching_waypoint_y == -0.643964) {
        map_chenge_flag[2] = false;
        map_chenge_flag[3] = true;
    } else if (switching_waypoint_x == 14.4161 && switching_waypoint_y == -2.06727 && switching_waypoint_y == -0.999967) {
        map_chenge_flag[3] = false;
        map_chenge_flag[4] = true;
    } else if (switching_waypoint_x == 13.8703 && switching_waypoint_y == 121.256 && switching_waypoint_y == 0.751127) {
        map_chenge_flag[4] = false;
        map_chenge_flag[5] = true;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "map_swithing");

    ros::NodeHandle n;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    ros::Subscriber map_to_baselink_tf_sub = n.subscribe("/map_to_base_link", 50, map_to_baselink_callback);
    ros::Subscriber waypoint_sub = n.subscribe("/waypoint", 10, waypoint_callback);

    ros::NodeHandle lSubscriber("");

    ros::Subscriber map_sub0 = lSubscriber.subscribe("/map0", 50, map_callback0);
    ros::Subscriber map_sub1 = lSubscriber.subscribe("/map1", 50, map_callback1);
    ros::Subscriber map_sub2 = lSubscriber.subscribe("/map2", 50, map_callback2);
    ros::Subscriber map_sub3 = lSubscriber.subscribe("/map3", 50, map_callback3);
    ros::Subscriber map_sub4 = lSubscriber.subscribe("/map4", 50, map_callback4);
    ros::Subscriber map_sub5 = lSubscriber.subscribe("/map5", 50, map_callback5);

    map_num.data = 0;

    ros::Rate loop_rate(50);

    while (n.ok()) {
        
        // referrence : https://monozukuri-c.com/langc-ifdef/
        #ifdef DEBUG_ON
            map_pub.publish(map_array.at(0));
            map_pub.publish(map_array.at(1));
            map_pub.publish(map_array.at(2));
            map_pub.publish(map_array.at(3));
            map_pub.publish(map_array.at(4));
            map_pub.publish(map_array.at(5));
        #else
            if (map_chenge_flag[0] == true) 
            {
                map_pub.publish(map_array.at(0));
            } 
            else if (map_chenge_flag[1] == true) 
            {
                map_pub.publish(map_array.at(1));
            } 
            else if (map_chenge_flag[2] == true) 
            {
                map_pub.publish(map_array.at(2));
            } 
            else if (map_chenge_flag[3] == true) 
            {
                map_pub.publish(map_array.at(3));
            } 
            else if (map_chenge_flag[4] == true) 
            {
                map_pub.publish(map_array.at(4));
            } 
            else if (map_chenge_flag[5] == true) 
            {
                map_pub.publish(map_array.at(5));
            }
        #endif

        ros::spinOnce();
        loop_rate.sleep();
    }
}