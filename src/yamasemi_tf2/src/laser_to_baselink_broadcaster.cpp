#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

std::string static_name;

int main(int argc, char **argv)
{
   ros::init(argc,argv, "laser_to_baselink");
   if(argc != 8)//引数の制限
   {
     ROS_ERROR("Invalid number of parameters\n usage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
     return -1;
   }
   if(strcmp(argv[1],"base_link")==0)//文字の比較関数
   {
     ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;
     }

    static_name = argv[1];
     static tf2_ros::StaticTransformBroadcaster static_broadcaster;//BroadCaster
     geometry_msgs::TransformStamped static_transformStamped;//BroadCastするメッセージ型
   
     static_transformStamped.header.stamp = ros::Time::now();
     static_transformStamped.header.frame_id = "base_link";//base_linkから見た...
     static_transformStamped.child_frame_id = static_name;//laser
     static_transformStamped.transform.translation.x = atof(argv[2]);//xyz空間におけるx値
     static_transformStamped.transform.translation.y = atof(argv[3]);
     static_transformStamped.transform.translation.z = atof(argv[4]);
     tf2::Quaternion quat;
     quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));//RPYからQuauternion計算
     static_transformStamped.transform.rotation.x = quat.x();
     static_transformStamped.transform.rotation.y = quat.y();
     static_transformStamped.transform.rotation.z = quat.z();
     static_transformStamped.transform.rotation.w = quat.w();
     static_broadcaster.sendTransform(static_transformStamped);//BroadCasterがメッセージをBroadcast
     ROS_INFO("Spinning until killed publishing laser to base_link");
     ros::spin();
     return 0;
   };