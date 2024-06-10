#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>

// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度

//std::map<std::string, double> params_; // パラメータをここに格納
std::map<std::string, std::string> params_;
geometry_msgs::PoseStamped goal1;	   // 目標地点
geometry_msgs::PoseStamped goal2;	   // 目標地点

// 初期速度
double v0 = 0.0;
// 初期角速度
double w0 = 0.0;

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
		//v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
		v = 0.2;
	else
		//v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
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

// 直線追従をするために指令する速度を計算してtwistに格納
void follow_line(double x, double y, double th)
{
	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	// パラメータファイルに入っている係数
	double k_eta = 800.00000000 / 100;
	double k_phai = 300.00000000 / 100;
	double k_w = 100.00000000 / 100;

	// ロボットの最大速度、最大角速度
	double v_max = 0.30000000;
	double w_max = 6.28000000;

	// ロボットと直線の距離(実際には一定以上の距離0.4でクリップ)
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(robot_x - x);
    else if (th == -M_PI / 2.0)
        eta = robot_x - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    if (eta > 0.40000000)
        eta = 0.40000000;
    else if (eta < -0.40000000)
        eta = -0.40000000;

	// 直線に対するロボットの向き(-M_PIからM_PIに収まるように処理)
	double phai = yaw - th;
	while (phai <= -M_PI || M_PI <= phai)
	{
		if (phai <= -M_PI)
			phai = phai + 2 * M_PI;
		else
			phai = phai - 2 * M_PI;
	}

	// 目標となるロボットの角速度と現在の角速度の差
	double w_diff = w0;

	// 角速度
	double w = w0 + (-k_eta * eta - k_phai * phai - k_w* w_diff);
	if (w > w_max)
		w = w_max;
	else if (w < -w_max)
		w = -w_max;

	// 並進速度
	double v = v0;
	if (v0 != 0)
		v = v0 - v0 / abs(v0) * abs(w0);
	if (v > v_max)
		v = v_max;
	else if (v < -v_max)
		v = -v_max;

	twist.linear.x = v;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w;

	// 現在の角速度を次の時間の計算に使用
	w0 = twist.angular.z;

	// デバック用のプリント
	// std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	// std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;
	// std::cout << "(x,y) = (" << robot_x << "," << robot_y << ")" << std::endl;
	// std::cout << "------------------------------" << std::endl;

	
}

void get_param(void){
	std::ifstream read_param_file;
	read_param_file.open(params_["param_file"], std::ios::in);

	std::string buffer_read;

	while(std::getline(read_param_file, buffer_read))
	{
		std::string row = "";
		std::string name_param = "";

        for (char &x : buffer_read)
        {
            if (x != ' ') {
                //add element to vector row
            std::string row_alpha{x};
			row = row + row_alpha;
			} else if(row != "" && name_param == ""){
				if ((row == "L_K1") | (row == "L_K2") | (row == "L_K3") | (row == "L_DIST") | (row == "MAX_VEL") | (row == "MAX_W")){
					name_param = row;
					row = "";
				}else{
					break;
				}
			} else if(row != ""){
				params_[name_param] = row;
				row = "";
				name_param = "";
			}

        }
	}
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
	ros::init(argc, argv, "kadai"); // ノード名の指定
	ros::NodeHandle nh;
	ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10);
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    nh.param<std::string>("param_file", params_["param_file"], "/home/s-k/researches/programs/platform/yp-robot-params/robot-params/speego.param");
	ros::Rate loop_rate(10); // １秒間にpublishする回数

    get_param();

	ros::Duration(5).sleep();
	ros::spinOnce();
	int count = 0;
	int start_i = 0;
	int end_i = 0;
	int obj1_i = 0;
	int obj2_i = 0;
	float object1[2];
	float object2[2];
	float road[2];
	bool continuity = false;

	for (int i = 0; i < 384 * 2; i++)
	{
		if (count == 0 && scan.ranges[i] <= 2.5 && continuity == false)
		{
			start_i = i;
			count++;
			continuity = true;
		}
		else if (count == 1 && scan.ranges[i] > 2.5 && continuity == true)
		{
			end_i = i;
			continuity = false;
			obj1_i = (start_i + end_i) / 2;
			object1[0] = scan.ranges[obj1_i] * sin(scan.angle_increment * obj1_i + scan.angle_min);
			object1[1] = scan.ranges[obj1_i] * cos(scan.angle_increment * obj1_i + scan.angle_min);
		}
		

		if (count == 1 && scan.ranges[i] <= 2.5 && continuity == false)
		{
			start_i = i;
			count++;
			continuity = true;
		}
		else if (count == 2 && scan.ranges[i] > 2.5 && continuity == true)
		{
			end_i = i;
			continuity = false;
			obj2_i = (start_i + end_i) / 2;
			object2[0] = scan.ranges[obj2_i] * sin(scan.angle_increment * obj2_i + scan.angle_min);
			object2[1] = scan.ranges[obj2_i] * cos(scan.angle_increment * obj2_i + scan.angle_min);
		}
	}

	road[0] = (object1[0] + object2[0]) / 2.0;
	road[1] = (object1[1] + object2[1]) / 2.0;

	// 通り抜けるように
	road[0] = road[0] * 1.5;
	road[1] = road[1] * 1.5;

	std::cout << "pole1.i: " << obj1_i << ", pole2.i: " << obj2_i << std::endl;
	std::cout << "pole.x: " << road[1] << ", pole.y: " << road[0] << std::endl;
	std::cout << "start: " << start_i << ", end: " << end_i << std::endl;
	
	scan_pub.publish(scan);


	// 2. ２つのゲートの間の角度を特定して直進する。
	// 3. そしてゴールへ

	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

	goal1.pose.position.x = road[1]; //特定したポールの位置の中間
	goal1.pose.position.y = road[0];
	goal2.pose.position.x = 5.0;
	goal2.pose.position.y = 0.0;
	int step = 0;

	while (ros::ok())
	{
		ros::spinOnce();
		
		if (step==0)
		{
			go_position(goal1);
			if (near_position(goal1))
			{
				twist.linear.x = 0.0;
				twist.angular.z = 0.0;
				step++;
			}

		}
		else if (step==1)
		{
			go_position(goal2);
			if (near_position(goal2))
			{
				twist.linear.x = 0.0;
				twist.angular.z = 0.0;
				step++;
			}
		}
		else
		{
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			break;
		}

		twist_pub.publish(twist);
		loop_rate.sleep();
	}
	
	std::cout << "done" << std::endl;

	return 0;
}
