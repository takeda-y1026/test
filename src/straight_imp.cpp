// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Quaternion.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/LaserScan.h>

// #include <map>
// #include <math.h>

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "straight_move"); // ノード名の指定
// 	ros::NodeHandle nh;
// 	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
// 	ros::Rate loop_rate(10); // １秒間にpublishする回数
//     ros::Time start_time;
//     geometry_msgs::Twist twist; // 指令する速度、角速度

// 	int step = 0;

// 	while (ros::ok()){		
// 		if (step==0){	
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.5)){ 
//                 twist.linear.x = 0.2;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(5.0)){ 
//                 twist.linear.x = 0.5;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.3)){ 
//                 twist.linear.x = 0.1;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.2)){ 
//                 twist.linear.x = 0.05;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             step++;
// 		}
// 		if (step==1){
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.5)){ 
//                 twist.linear.x = -0.2;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(5.0)){ 
//                 twist.linear.x = -0.5;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.3)){ 
//                 twist.linear.x = -0.1;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             start_time = ros::Time::now();
//             while (ros::Time::now() - start_time < ros::Duration(0.2)){ 
//                 twist.linear.x = -0.05;
//                 twist.angular.z = 0.0;
//                 twist_pub.publish(twist);
// 		        loop_rate.sleep();
//             }
//             step--;
// 		}
// 	}

// 	return 0;
// }



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "straight_move"); // ノード名の指定
    ros::NodeHandle nh;
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Rate loop_rate(10); // １秒間にpublishする回数

    // 定常時の速さの変数
    double constant_linear_speed = 0.5;

    int step = 0;
    ros::Time start_time;
    geometry_msgs::Twist twist; // 指令する速度、角速度

    while (ros::ok())
    {
        if (step == 0)
        {
            // 加速フェーズ
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(2.0))
            {
                double t = (ros::Time::now() - start_time).toSec();
                twist.linear.x = constant_linear_speed * (t / 2.0);
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 定常フェーズ
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(5.0))
            {
                twist.linear.x = constant_linear_speed;
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 減速フェーズ
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(2.0))
            {
                double t = (ros::Time::now() - start_time).toSec();
                twist.linear.x = constant_linear_speed * (1.0 - t / 2.0);
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 停止フェーズ
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            twist_pub.publish(twist);
            loop_rate.sleep();

            step++;
        }
        else if (step == 1)
        {
            // 加速フェーズ（逆方向）
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(2.0))
            {
                double t = (ros::Time::now() - start_time).toSec();
                twist.linear.x = -constant_linear_speed * (t / 2.0);
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 定常フェーズ（逆方向）
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(5.0))
            {
                twist.linear.x = -constant_linear_speed;
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 減速フェーズ（逆方向）
            start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(2.0))
            {
                double t = (ros::Time::now() - start_time).toSec();
                twist.linear.x = -constant_linear_speed * (1.0 - t / 2.0);
                twist.angular.z = 0.0;
                twist_pub.publish(twist);
                loop_rate.sleep();
            }

            // 停止フェーズ
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            twist_pub.publish(twist);
            loop_rate.sleep();

            step--;
        }
    }

    return 0;
}
