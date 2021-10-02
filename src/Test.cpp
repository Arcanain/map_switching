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
/*
#include <stdint.h>
#include <map>
*/
#include "Vector.h"
#include "tf_lis.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

const int select_maps = 6;

vector<nav_msgs::OccupancyGrid> map_array(select_maps);

std_msgs::Int32 map_num;

/*
// Description for a single map cell.
typedef struct
{
// Occupancy state (-1 = free, 0 = unknown, +1 = occ)
int occ_state;

// Distance to the nearest occupied cell
double occ_dist;

// Wifi levels
//int wifi_levels[MAP_WIFI_MAX_LEVELS];
} map_cell_t;

// Description for a map
typedef struct
{
// Map origin; the map is a viewport onto a conceptual larger map.
double origin_x, origin_y;

// Map scale (m/cell)
double scale;

// Map dimensions (number of cells)
int size_x, size_y;
 
// The map data, stored as a grid
map_cell_t *cells;

// Max distance at which we care about obstacles, for constructing
// likelihood field
double max_occ_dist;

} map_t;
*/

// 綺麗な形で重ならなくても、ロボットの自己位置情報からmapの原点をオフセットすれば問題ないのでは？
// https://daily-tech.hatenablog.com/entry/2019/11/25/062219
// tf_lis lidar_tf("/map","/base_link")を利用して、map->base_link間の座標変換からmap上でのbase_linkの位置情報を取得する

// map0 origin
float map0_origin_x = -100.000000;
float map0_origin_y = -100.000000;
float map0_orientation_z = 0.000000;

//map0 final waypoint
float map0_final_origin_x = -24.4127;
float map0_final_origin_y = -28.9142;
float map0_final_orientation_z = -0.999601;

//map1 final waypoint
float map1_final_origin_x = 203.174;
float map1_final_origin_y = -82.518;
float map1_final_orientation_z = -0.721927;

//map2 final waypoint
float map2_final_origin_x = 5.92503;
float map2_final_origin_y = 148.57;
float map2_final_orientation_z = -0.643964;

//map3 final waypoint
float map3_final_origin_x = 14.4161;
float map3_final_origin_y = -2.06727;
float map3_final_orientation_z = -0.999967;

//map4 final waypoint
float map4_final_origin_x = 13.8703;
float map4_final_origin_y = 121.256;
float map4_final_orientation_z = 0.751127;

//map5 final waypoint
float map5_final_origin_x = - 24.4127;
float map5_final_origin_y = - 28.9142;
float map5_final_orientation_z = - 0.999601;

// map0
void map_callback0(const nav_msgs::OccupancyGrid& map_msg) {
    map_array.at(0) = map_msg;
}

// map1
void map_callback1(const nav_msgs::OccupancyGrid& map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;

    //map.info.origin.position.x = map0_origin_x + map0_final_origin_x;
    //map.info.origin.position.y = map0_origin_y + map0_final_origin_y;
    //map.info.origin.position.x = map_msg.info.origin.position.x - 24.4127;
    //map.info.origin.position.y = map_msg.info.origin.position.y - 28.9142;
    //map.info.origin.position.x = -24.4127;
    //map.info.origin.position.y = -28.9142;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    //map.info.origin.orientation.w = - 0.999601;
    //map.info.origin.orientation.w = map_msg.info.origin.orientation.z - 0.999601;
    //map.info.origin.orientation.w = map0_orientation_z + map0_final_orientation_z;
    map.data = map_msg.data;

    map_array.at(1) = map;
}

//map2
void map_callback2(const nav_msgs::OccupancyGrid& map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;

    //map.info.origin.position.x = map0_origin_x + map0_final_origin_x + map1_final_origin_x;
    //map.info.origin.position.y = map0_origin_y + map0_final_origin_y + map1_final_origin_y;
    //map.info.origin.position.x = map_msg.info.origin.position.x + 203.174;
    //map.info.origin.position.y = map_msg.info.origin.position.y - 82.518;
    //map.info.origin.position.x = 203.174;
    //map.info.origin.position.y = -82.518;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    //map.info.origin.orientation.w = map_msg.info.origin.orientation.z - 0.721927;
    //map.info.origin.orientation.w = map0_orientation_z + map0_final_orientation_z + map1_final_orientation_z;
    map.data = map_msg.data;

    map_array.at(2) = map;
}

//map3
void map_callback3(const nav_msgs::OccupancyGrid& map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;

    //map.info.origin.position.x = map0_origin_x + map0_final_origin_x + map1_final_origin_x + map2_final_origin_x;
    //map.info.origin.position.y = map0_origin_y + map0_final_origin_y + map1_final_origin_y + map2_final_origin_y;
    //map.info.origin.position.x = map_msg.info.origin.position.x + 203.174;
    //map.info.origin.position.y = map_msg.info.origin.position.y - 82.518;
    //map.info.origin.position.x = 203.174;
    //map.info.origin.position.y = -82.518;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    //map.info.origin.orientation.w = map_msg.info.origin.orientation.z - 0.721927;
    //map.info.origin.orientation.w = map0_orientation_z + map0_final_orientation_z + map1_final_orientation_z + map2_final_orientation_z;
    map.data = map_msg.data;

    map_array.at(3) = map;
}

//map4
void map_callback4(const nav_msgs::OccupancyGrid& map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;

    //map.info.origin.position.x = map0_origin_x + map0_final_origin_x + map1_final_origin_x + map2_final_origin_x + map3_final_origin_x;
    //map.info.origin.position.y = map0_origin_y + map0_final_origin_y + map1_final_origin_y + map2_final_origin_y + map3_final_origin_y;
    //map.info.origin.position.x = map_msg.info.origin.position.x + 203.174;
    //map.info.origin.position.y = map_msg.info.origin.position.y - 82.518;
    //map.info.origin.position.x = 203.174;
    //map.info.origin.position.y = -82.518;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    //map.info.origin.orientation.w = map_msg.info.origin.orientation.z - 0.721927;
    //map.info.origin.orientation.w = map0_orientation_z + map0_final_orientation_z + map1_final_orientation_z + map2_final_orientation_z + map3_final_orientation_z;
    map.data = map_msg.data;

    map_array.at(4) = map;
}

//map5
void map_callback5(const nav_msgs::OccupancyGrid& map_msg) {
    nav_msgs::OccupancyGrid map;
    
    map.info.map_load_time = ros::Time::now();
    map.info.width = map_msg.info.width;
    map.info.height = map_msg.info.height;
    map.info.resolution = map_msg.info.resolution;

    //map.info.origin.position.x = map0_origin_x + map0_final_origin_x + map1_final_origin_x + map2_final_origin_x + map3_final_origin_x + map4_final_origin_x;
    //map.info.origin.position.y = map0_origin_y + map0_final_origin_y + map1_final_origin_y + map2_final_origin_y + map3_final_origin_y + map4_final_origin_y;
    //map.info.origin.position.x = map_msg.info.origin.position.x + 203.174;
    //map.info.origin.position.y = map_msg.info.origin.position.y - 82.518;
    //map.info.origin.position.x = 203.174;
    //map.info.origin.position.y = -82.518;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    //map.info.origin.orientation.w = map_msg.info.origin.orientation.z - 0.721927;
    //map.info.origin.orientation.w = map0_orientation_z + map0_final_orientation_z + map1_final_orientation_z + map2_final_orientation_z + map3_final_orientation_z + map4_final_orientation_z;
    map.data = map_msg.data;

    map_array.at(5) = map;
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

void tf_callback(const tf2_msgs::TFMessage &msg) {

}

void map_to_baselink_callback(const geometry_msgs::Pose &msg) {

}

int main(int argc, char **argv){
    ros::init(argc, argv, "Test");

    ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    ros::Subscriber map_to_baselink_tf_sub = n.subscribe("/map_to_base_link", 50, map_to_baselink_callback);
    ros::ServiceClient map_client = n.serviceClient<nav_msgs::SetMap>("set_map");

    ros::NodeHandle lSubscriber("");

    ros::Subscriber map_sub0 = lSubscriber.subscribe("/map0", 50, map_callback0);
    ros::Subscriber map_sub1 = lSubscriber.subscribe("/map1", 50, map_callback1);
    ros::Subscriber map_sub2 = lSubscriber.subscribe("/map2", 50, map_callback2);
    ros::Subscriber map_sub3 = lSubscriber.subscribe("/map3", 50, map_callback3);
    ros::Subscriber map_sub4 = lSubscriber.subscribe("/map4", 50, map_callback4);
    ros::Subscriber map_sub5 = lSubscriber.subscribe("/map5", 50, map_callback5);
    
    //ros::Subscriber tf_sub = lSubscriber.subscribe("/tf", 50, tf_callback);
    //ros::Subscriber map_to_baselink_tf_sub = lSubscriber.subscribe<geometry_msgs::Pose>("/map_to_base_link", 50, map_to_baselink_callback);

    //Now wp publisher
    ros::Publisher map_chenged_pub = n.advertise<std_msgs::Empty>("map_chenged", 10);
    //tf_lis lidar_tf("map","base_link");

    map_num.data = 0;
    int count = 0;
    ros::Rate loop_rate(50);

    /*
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped pose_stamped;
    tf::TransformListener listener;
    tf::StampedTransform trans_slam;
    listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    */

    while (n.ok()) {
        //lidar_tf.update();
        
        /*
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
        */

        //１．マップ１でロボットの自己位置推定をして，マップ２の作成を開始した点まで来たら位置を取得する．
        //２．１で取得した位置をマップ２のオフセットとして使用する．

        /* map initial origin
        Vector diff_vec=lidar_tf.pos-now_wp_vec;
        double diff_rad=next_wp_vec.yaw-now_wp_vec.yaw;
        diff_vec=diff_vec.rad_rot(diff_rad);
        Vector new_vec=next_wp_vec+diff_vec;
        new_vec.yaw=lidar_tf.pos.yaw+diff_rad;
        set_map.request.initial_pose=vec_to_PoseWithCovarianceStamped(new_vec);
        */
        
        /*
        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped pose_stamped;
        tf::TransformListener listener;
        tf::StampedTransform trans_slam;
        */

        /*
        try {
            //listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(0.1));
            //listener.lookupTransform("base_link", "map",ros::Time(0), trans_slam);
            listener.lookupTransform("map", "base_link",ros::Time(0), trans_slam);
            pose.position.x=trans_slam.getOrigin().x();
            pose.position.y=trans_slam.getOrigin().y();
            pose.position.z=trans_slam.getOrigin().z();
            pose.orientation.x=trans_slam.getRotation().x();
            pose.orientation.y=trans_slam.getRotation().y();
            pose.orientation.z=trans_slam.getRotation().z();
            pose.orientation.w=trans_slam.getRotation().w();
            pose_stamped.header.frame_id="map";
            pose_stamped.header.stamp=ros::Time::now();
            pose_stamped.pose=pose;
        }
        catch (tf::TransformException &ex)  {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        */

        map_pub.publish(map_array.at(0));
        map_pub.publish(map_array.at(1));
        map_pub.publish(map_array.at(2));
        map_pub.publish(map_array.at(3));
        map_pub.publish(map_array.at(4));
        map_pub.publish(map_array.at(5));

        ros::spinOnce();
        loop_rate.sleep();

        count++;
    }
}