/*
 * ********************************************************************************
 * SYSTEM | ロボットの現在位置(map->base_link)とwaypointを基にmapを切り替える
 * ********************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
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

/***********************************************************************
 * Global variables
 **********************************************************************/
const int select_maps = 6;

vector<nav_msgs::OccupancyGrid> map_array(select_maps);

std_msgs::Int32 map_num;

geometry_msgs::Pose current_robot_pose;

// 各種map切り替え時のx, y座標
float switching_waypoint_x;
float switching_waypoint_y;
float switching_waypoint_w;
bool map_chenge_flag[6] = {true, false, false, false, false, false};
bool map_reconstraction_flag[6] = {false, false, false, false, false, false};

// map原点の保存
geometry_msgs::Pose map_origin;

/***********************************************************************
 * Function declerations
 **********************************************************************/
// 各種mapデータの原点を現時点でのロボットの位置を原点に変換する処理
nav_msgs::OccupancyGrid map_reconstruction(nav_msgs::OccupancyGrid map_msg, bool reconstruction_flag) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;
    
    // map原点の切り替えは各mapにつき一回のみでOK
    if (reconstruction_flag == false) {
        map.info.origin.position.x = current_robot_pose.position.x;
        map.info.origin.position.y = current_robot_pose.position.y;
        map.info.origin.position.z = current_robot_pose.position.z;
        map.info.origin.orientation.x = current_robot_pose.orientation.x;
        map.info.origin.orientation.y = current_robot_pose.orientation.y;
        map.info.origin.orientation.z = current_robot_pose.orientation.z;
        map.info.origin.orientation.w = current_robot_pose.orientation.w;

        // map原点の保存
        map_origin.position.x = map.info.origin.position.x;
        map_origin.position.y = map.info.origin.position.y;
        map_origin.position.z = map.info.origin.position.z;
        map_origin.orientation.x = map.info.origin.orientation.x;
        map_origin.orientation.y = map.info.origin.orientation.y;
        map_origin.orientation.z = map.info.origin.orientation.z;
        map_origin.orientation.w = map.info.origin.orientation.w;
    }

    map.data = map_msg.data;

    return map;
}

// 各種mapを事前に配列に格納しておく
// map0
void map_callback0(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(0) = map_msg;
}

// map1
void map_callback1(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(1) = map_msg;
}

//map2
void map_callback2(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(2) = map_msg;
}

//map3
void map_callback3(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(3) = map_msg;
}

//map4
void map_callback4(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(4) = map_msg;
}

//map5
void map_callback5(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(5) = map_msg;
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

// ロボットの現在位置(map->base_link)を取得
void map_to_baselink_callback(const geometry_msgs::Pose &msg) {
    current_robot_pose.position.x = msg.position.x;
    current_robot_pose.position.y = msg.position.y;
    current_robot_pose.position.z = msg.position.z;
    current_robot_pose.orientation.x = msg.orientation.x;
    current_robot_pose.orientation.y = msg.orientation.y;
    current_robot_pose.orientation.z = msg.orientation.z;
    current_robot_pose.orientation.w = msg.orientation.w;
}

// 各種mapのwaypointの最終点に到達した段階でchange flagを切り替える
void waypoint_callback(const visualization_msgs::Marker &msg) {
    switching_waypoint_x = msg.pose.position.x;
    switching_waypoint_y = msg.pose.position.y;
    switching_waypoint_w = msg.pose.orientation.w;

    // それぞれの条件の値は各mapのwaypointの最終ポイント
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

// 原点を現在のロボット位置に切り替える
nav_msgs::OccupancyGrid map_switch(nav_msgs::OccupancyGrid temp_map, bool reconstraction_flag, int map_num, int count) {
    nav_msgs::OccupancyGrid map;
    map = map_reconstruction(temp_map, reconstraction_flag);
    
    // 現時点ではcountの値でtrueにしているが、本来は切り替えた時点でtrueにする。
    // おそらく以下の処理で上手く動作する。
    //if (reconstraction_flag == false) {
    //    map_reconstraction_flag[map_num] = true;
    //}
    if (count > 100) {
        map_reconstraction_flag[map_num] = true;
    }

    if (reconstraction_flag == true) {
        map.info.origin.position.x = map_origin.position.x;
        map.info.origin.position.y = map_origin.position.y;
        map.info.origin.position.z = map_origin.position.z;
        map.info.origin.orientation.x = map_origin.orientation.x;
        map.info.origin.orientation.y = map_origin.orientation.y;
        map.info.origin.orientation.z = map_origin.orientation.z;
        map.info.origin.orientation.w = map_origin.orientation.w;
    }

    return map;
}

/**
 *****************************************************************************************************
 * main 
 *****************************************************************************************************
 */
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
    
    int count = 0;
    int map_num = 0;
    
    ros::Rate loop_rate(50);

    while (n.ok()) {
        
        // referrence : https://monozukuri-c.com/langc-ifdef/
        #ifdef DEBUG_ON
            // デバックパターン1
            /*
            map_pub.publish(map_array.at(0));
            map_pub.publish(map_array.at(1));
            map_pub.publish(map_array.at(2));
            map_pub.publish(map_array.at(3));
            map_pub.publish(map_array.at(4));
            map_pub.publish(map_array.at(5));
            */
            
            // デバックパターン2
            /*
            nav_msgs::OccupancyGrid map;
            nav_msgs::OccupancyGrid temp_map;
            bool reconstraction_flag;
            temp_map = map_array.at(1);
            reconstraction_flag = map_reconstraction_flag[1];
            map_num = 1;
            map = map_switch(temp_map, reconstraction_flag, map_num, count);

            map_pub.publish(map);
            */

            // デバックパターン3
            nav_msgs::OccupancyGrid map;
            nav_msgs::OccupancyGrid temp_map;
            bool reconstraction_flag;
            
            if (count >= 0 && count < 50) {
                map_num = 0;
                temp_map = map_array.at(map_num);
                map = temp_map;
            } else if (count >= 50 && count < 100) {
                map_num = 1;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } else if (count >= 100 && count < 150) {
                map_num = 2;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } else if (count >= 150 && count < 200) {
                map_num = 3;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } else if (count >= 200 && count < 250) {
                map_num = 4;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } else if (count >= 250 && count < 300) {
                map_num = 5;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } else if (count >= 300) {
                map_num = 0;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            }

            map_pub.publish(map);

        #else
            nav_msgs::OccupancyGrid map;
            nav_msgs::OccupancyGrid temp_map;
            bool reconstraction_flag;

            if (map_chenge_flag[0] == true) 
            {
                map_num = 0;
                temp_map = map_array.at(map_num);
                map = temp_map;
            } 
            else if (map_chenge_flag[1] == true) 
            {
                map_num = 1;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } 
            else if (map_chenge_flag[2] == true) 
            {
                map_num = 2;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } 
            else if (map_chenge_flag[3] == true) 
            {
                map_num = 3;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } 
            else if (map_chenge_flag[4] == true) 
            {
                map_num = 4;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            } 
            else if (map_chenge_flag[5] == true) 
            {
                map_num = 5;
                temp_map = map_array.at(map_num);
                reconstraction_flag = map_reconstraction_flag[map_num];
                map = map_switch(temp_map, reconstraction_flag, map_num, count);
            }

            map_pub.publish(map);
        #endif

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}