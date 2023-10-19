#include<ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h" 
using namespace std;
//角度制转弧度制
double rad(double d)
{
	return d * 3.14159265 / 180.0;
}
// ROS订阅者和发布者
ros::Subscriber gps_data_sub;
// ros::Publisher path_pub;
ros::Publisher xyz_pub;
ros::Publisher chatter_pub;

// 全局变量
string gps_sub_topic = "";
string output_frame_name ="";
double z_rotate_value = 0.0;
double origin_latitude_value = 0.0;
double origin_longitude_value = 0.0;
double origin_altitude_value = 0.0;
double latitude_resolution = 0.0;
double longitude_resolution = 0.0;
double altitude_resolution = 0.0;
bool init_flag = true;
nav_msgs::Path path;
void gps_to_xyz(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    if (init_flag == true)
    {
        origin_latitude_value = gps_msg->latitude;
        origin_longitude_value = gps_msg->longitude;
        origin_altitude_value = gps_msg->altitude;
        init_flag = false;
    }
    else
    {
        double gps_lat = gps_msg->latitude;
        double gps_lon = gps_msg->longitude;
        double gps_hei = gps_msg->altitude;
        double gps_xx = (gps_lon - origin_longitude_value) / longitude_resolution;
        double gps_yy = (gps_lat - origin_latitude_value) / latitude_resolution;
        double gps_zz = (gps_hei - origin_altitude_value) / altitude_resolution;
        double gps_x = cos(rad(z_rotate_value)) * gps_xx + (-sin(rad(z_rotate_value))) * gps_yy;
        double gps_y = sin(rad(z_rotate_value)) * gps_xx + cos(rad(z_rotate_value)) * gps_yy;
        double gps_z = gps_zz;
 
        // if (path_pub.getNumSubscribers() > 0)
        // {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.header.frame_id = output_frame_name;
            this_pose_stamped.header.seq = gps_msg->header.seq;
            this_pose_stamped.header.stamp = gps_msg->header.stamp;
            this_pose_stamped.pose.position.x = gps_x;
            this_pose_stamped.pose.position.y = gps_y;
            this_pose_stamped.pose.position.z = gps_z;
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            path.poses.push_back(this_pose_stamped);
            path.header.frame_id = output_frame_name;
        // }
        // path_pub.publish(path);
            xyz_pub.publish(this_pose_stamped);
            std_msgs::String msg;//建立暂存区，先将消息放入，在进行publish
            msg.data = "wkkzmgs";
            chatter_pub.publish(msg);
    }
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"gps_deal");
    ros::NodeHandle nh;
    xyz_pub = nh.advertise<geometry_msgs::PoseStamped>("/ori_gps2xyz", 5, true);
    nh.param<string>("gps_sub_topic", gps_sub_topic, "/gps");
    nh.param<string>("output_frame_name", output_frame_name, "map");
    nh.param<bool>("auto_get_origin_gps", init_flag, true);
    nh.param<double>("z_rotate_value", z_rotate_value, 1.0);
    nh.param<double>("origin_latitude_value", origin_latitude_value, 1.0);
    nh.param<double>("origin_longitude_value", origin_longitude_value, 1.0);
    nh.param<double>("origin_altitude_value", origin_altitude_value, 1.0);
    nh.param<double>("latitude_resolution", latitude_resolution, 1.0);
    nh.param<double>("longitude_resolution", longitude_resolution, 1.0);
    nh.param<double>("altitude_resolution", altitude_resolution, 1.0);
    gps_data_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_sub_topic, 1000, gps_to_xyz);
    // path_pub = nh.advertise<nav_msgs::Path>("/gpsTrack",1, true);
    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    
    ros::spin();
    return 0;
}