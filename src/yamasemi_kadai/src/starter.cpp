#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <std_msgs/Time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "starter");
  ros::NodeHandle nh;
  ros::Publisher start_time_pub = nh.advertise<std_msgs::Time>("start_time", 10);
  ros::Rate loop_rate(10);
  ros::Time ros_start_time = ros::Time::now();
  
  while (ros::ok())
  {
    std_msgs::Time start_time;
    start_time.data = ros_start_time;
    start_time_pub.publish(start_time);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}