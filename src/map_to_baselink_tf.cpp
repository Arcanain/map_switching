#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "Vector.h"
#include "tf_lis.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "map_to_baselink_tf");

    ros::NodeHandle n;
    ros::Publisher map_to_baselink_tf = n.advertise<geometry_msgs::Pose>("/map_to_base_link", 10);

    tf_lis lidar_tf("map","base_link");

    ros::Rate loop_rate(50);

    while (n.ok()) {
        lidar_tf.update();
        
        geometry_msgs::Pose pose;
        pose = lidar_tf.pose;
        map_to_baselink_tf.publish(pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
}