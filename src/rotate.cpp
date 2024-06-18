#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <map>
#include <math.h>

// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度

int main(int argc, char **argv)
{
	// 1. ゲートの位置(角度、距離)を特定
	// printf("123");
	ros::init(argc, argv, "rotate"); // ノード名の指定
	ros::NodeHandle nh;
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
	ros::Rate loop_rate(10); // １秒間にpublishする回数
	//ros::Time start_time;
	
	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

	int step = 0;

	while (ros::ok()){		
		if (step==0){
            twist.linear.x = 0.0;
            twist.angular.z = 0.5;			
		}
		// if (step==1){	
        //     twist.linear.x = 0.0;
        //     twist.angular.z = -0.5;
		// }

		twist_pub.publish(twist);
		loop_rate.sleep();
	}

	return 0;
}
