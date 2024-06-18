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

geometry_msgs::PoseStamped current_goal; // 目標地点

// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

// クォータニオンをオイラーに変換
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// 　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.4);
}

// 2つの円の中心と半径
double circle_center1_x = -3.0;
double circle_center1_y = 0.0;
double circle_center2_x = 3.0;
double circle_center2_y = 0.0;
double circle_radius = 3.0;
double angular_velocity = 0.5; // 円周上を移動する角速度

void update_goal(){
    // 現在のロボットの位置から目標地点を計算
    double theta = 0.0;
    bool reverse = false;
	bool second_circle = false; // 第二の円を描くフラグ

    if (reverse){
        theta -= angular_velocity * 0.1; // 0.1秒ごとに角度を更新
        if (theta <= 0){
            theta = 0;
            reverse = false;
			second_circle = !second_circle;
        }
    }
    else{
        theta += angular_velocity * 0.1; // 0.1秒ごとに角度を更新
        if (theta >= M_PI){
            theta = M_PI;
            reverse = true;
			second_circle = !second_circle;
        }
    }
	if (second_circle){
        current_goal.pose.position.x = circle_center2_x + circle_radius * cos(theta);
        current_goal.pose.position.y = circle_center2_y + circle_radius * sin(theta);
    }
    else{
        current_goal.pose.position.x = circle_center1_x + circle_radius * cos(theta);
        current_goal.pose.position.y = circle_center1_y + circle_radius * sin(theta);
    }
}

void go_position(geometry_msgs::PoseStamped goal)
{
	double k_v = 0.1; // 速度の係数
	double k_w = 1.6; // 角速度の係数

	// 指令する速度と角速度
	double v = 0.0;
	double w = 0.0;

	// 　角速度の計算
	double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	while (yaw <= -M_PI || M_PI <= yaw)
	{
		if (yaw <= -M_PI)
			yaw = yaw + 2 * M_PI;
		else
			yaw = yaw - 2 * M_PI;
	}

	theta = theta - yaw; // thetaに目標とする点に向くために必要となる角度を格納

	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	w = k_w * theta;

	// 速度の計算(追従する点が自分より前か後ろかで計算を変更)
	if (theta <= M_PI / 2 && theta >= -M_PI / 2)
		v = 0.2;
	else
		v = -0.2;

	// publishする値の格納
	twist.linear.x = v; // 速さ
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w; // 角速度

	// std::cout << "v: " << v << ", w: " << w << std::endl;
}

sensor_msgs::LaserScan scan;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
	float nandeyanen = scan_->range_max;
	scan.header = scan_->header;
	scan.angle_min = scan_->angle_min;
	scan.angle_max = scan_->angle_max;
	scan.angle_increment = scan_->angle_increment;
	scan.time_increment = scan_->time_increment;
	scan.scan_time = scan_->scan_time;
	scan.range_min = scan_->range_min;
	scan.range_max = scan_->range_max;
	scan.ranges = scan_->ranges;
	scan.intensities = scan_->intensities;
}

int main(int argc, char **argv)
{
	// 1. ゲートの位置(角度、距離)を特定
	// printf("123");
	ros::init(argc, argv, "wiper"); // ノード名の指定
	ros::NodeHandle nh;
	ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10); //<message>("topic", buffer);
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
	ros::Rate loop_rate(10); // １秒間にpublishする回数

	ros::Duration(5).sleep();
	ros::spinOnce();

	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

	while (ros::ok()){
		ros::spinOnce();
		
		update_goal();
		go_position(current_goal);

		twist_pub.publish(twist);
		loop_rate.sleep();
	}
	return 0;
}
