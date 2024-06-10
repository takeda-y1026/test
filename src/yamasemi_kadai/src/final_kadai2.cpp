#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <ros/time.h>
#include <std_msgs/Time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <map>
#include <math.h>

// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y; //ワールド座標系
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度
geometry_msgs::PoseStamped goal1;	   // 目標地点
geometry_msgs::PoseStamped goal2;
geometry_msgs::PoseStamped goal3;	 
geometry_msgs::PoseStamped goal4;
geometry_msgs::PoseStamped goal5;	  
geometry_msgs::PoseStamped midPoint1;
geometry_msgs::PoseStamped midPoint2; 
geometry_msgs::PoseStamped midPoint3; 


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
	double k_v = 0.5; // 速度の係数
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
		v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
		//v = 0.3;
	else
		v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
		//v = -0.3;
 
    if (v > 0.3) v = 0.3;
    if (v < -0.3) v = -0.3;
    //if (w > 1.0) w = 1.0;
    //if (w < -1.0) w = -1.0;

	// publishする値の格納
	twist.linear.x = v; // 速さ
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w; // 角速度

	// std::cout << "v: " << v << ", w: " << w << std::endl;
}

// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
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


int main(int argc, char** argv){
    std::cout << " acde " << std::endl;
    ros::init(argc, argv, "final_kadai2");
    ros::NodeHandle nh;
	ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    //ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10); //<message>("topic", buffer);
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);    
    ros::Rate rate(10.0);
	ros::Duration(3.0).sleep();
    enum State {RUN, STOP, AVOID, GOAL} state;
    ros::spinOnce();
    bool isObstacle = false;
    int rateCount = 0;
    
    goal1.pose.position.x = 4.5; //ゴールの場所をセット
	goal1.pose.position.y = 0.0;
	goal2.pose.position.x = 4.5;
	goal2.pose.position.y = 1.0;
    goal3.pose.position.x = 3.0;
	goal3.pose.position.y = -1.0;
	goal4.pose.position.x = 1.0;
	goal4.pose.position.y = 1.0;
    goal5.pose.position.x = 0.0;
	goal5.pose.position.y = 1.0;
    int center = scan.ranges.size() / 2;
    int step = 0;
    int avoid_step = 0;
    //ros::Time start_time;	
    state = RUN; //スタート  

    //std::cout << " abcde " << std::endl;  

	while (ros::ok()){	
        ros::spinOnce();
        isObstacle = false;
        for (int i = center - 50; i < center + 50; i++){
            if (scan.ranges[i] < 1.0){
                isObstacle = true;
            }
        }
        //std::cout << "obstacle1: "  << isObstacle << std::endl;
        switch (state){
            case RUN: //自律移動
                ROS_INFO("RUN\n");

                if (step==0){
                    go_position(goal1);
                    if (near_position(goal1)){
                        //twist.linear.x = 0.0;
                        //twist.angular.z = 0.0;
                        step++;
                    }
                }
                else if (step==1){
                    go_position(goal2);
                    if (near_position(goal2)){
                        //twist.linear.x = 0.0;
                        //twist.angular.z = 0.0;
                        step++;
                    }
                }
                else if (step==2){
                    go_position(goal3);
                    if (near_position(goal3)){
                        //twist.linear.x = 0.0;
                        //twist.angular.z = 0.0;
                        step++;
                    }
                }
                else if (step==3){
                    go_position(goal4);
                    if (near_position(goal4)){
                        //twist.linear.x = 0.0;
                        //twist.angular.z = 0.0;
                        step++;
                    }
                }
                else if (step==4){
                    go_position(goal5);
                    if (near_position(goal5)){
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        ROS_INFO("RUN -> GOAL\n");
                        state = GOAL;
                        break;
                    }
                }

                //waypointに沿って動くが障害物が眼の前にある場合はSTOP
                if (isObstacle){
                    rateCount = 0;
                    ROS_INFO("RUN -> STOP\n");
                    state = STOP;
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    break; //RUN内のwhileループから抜ける
                }
                break;  

            case STOP: //一時停止
                ROS_INFO("STOP\n");
                if (rateCount < 30){ //最大３秒間待機
                    //std::cout << "obstacle2: " << isObstacle << std::endl;
                    if (!isObstacle){ //障害物が取り除かれた
                        ROS_INFO("STOP -> RUN\n");
                        state = RUN;
                        break;
                    }
                    rateCount++;
                }
                else {
                    ROS_INFO("STOP -> AVOID\n");
                    state = AVOID; //取り除かれなかった
                    rateCount = 0;
                }
                break;
            
            case AVOID: //障害物回避
                ROS_INFO("AVOID\n");
                if (rateCount > 30){
                    avoid_step++;
                    rateCount = 0;
                }
                // if (avoid_step==0){
                //     midPoint1.pose.position.x = robot_x;
                //     midPoint1.pose.position.y = robot_y + 1.0;
                //     midPoint2.pose.position.x = robot_x + 1.0;
                //     midPoint2.pose.position.y = robot_y + 1.0;
                //     midPoint3.pose.position.x = robot_x + 1.0;
                //     midPoint3.pose.position.y = robot_y;
                //     avoid_step++;
                // }
                // else if (avoid_step==1){
                //     go_position(midPoint1);
                //     ROS_INFO("midPoint1\n");
                //     if (near_position(midPoint1)){
                //         //twist.linear.x = 0.0;
                //         //twist.angular.z = 0.0;
                //         ROS_INFO("near_midPoint1\n");
                //         avoid_step++;
                //     }
                // }
                // else if (avoid_step==2){
                //     go_position(midPoint2);
                //     ROS_INFO("midPoint2\n");
                //     if (near_position(midPoint2)){
                //         //twist.linear.x = 0.0;
                //         //twist.angular.z = 0.0;
                //         ROS_INFO("near_midPoint2\n");
                //         avoid_step++;
                //     }
                // }
                // else if (avoid_step==3){
                //     go_position(midPoint3);
                //     ROS_INFO("midPoint3\n");
                //     if (near_position(midPoint3)){
                //         //twist.linear.x = 0.0;
                //         //twist.angular.z = 0.0;
                //         ROS_INFO("near_midPoint3\n");
                //         avoid_step++;
                //     }
                // }
                // else {
                //     //物体を避けたら
                //     ROS_INFO("AVOID -> RUN\n");
                //     avoid_step = 0;
                //     state = RUN; 
                //     break;
                // }
                if (avoid_step==0 && rateCount<=30){
                    ROS_INFO("rotate1\n");
                    twist.angular.z = M_PI / 6.0; // 角速度
                    twist.linear.x = 0.0;
                    rateCount++;
                    break;
                }

                if (avoid_step==1 && rateCount<=30){
                    ROS_INFO("straight1\n");
                    twist.angular.z = 0.0; // 角速度
                    twist.linear.x = 0.20;
                    rateCount++;
                    break;
                }

                if (avoid_step==2 && rateCount<=30){
                    ROS_INFO("rotate2\n");
                    twist.angular.z = -M_PI / 6.0; // 角速度
                    twist.linear.x = 0.0;
                    rateCount++;
                    break;
                }

                if (avoid_step==3 && rateCount<=30){
                    ROS_INFO("straight2\n");
                    twist.angular.z = 0.0; // 角速度
                    twist.linear.x = 0.33;
                    rateCount++;
                    break;
                }
                if (avoid_step==4 && rateCount<=30){
                    ROS_INFO("straight2-2\n");
                    twist.angular.z = 0.0; // 角速度
                    twist.linear.x = 0.33;
                    rateCount++;
                    break;
                }

                if (avoid_step==5 && rateCount<=30){
                    ROS_INFO("rotate3\n");
                    twist.angular.z = -M_PI / 6.0; // 角速度
                    twist.linear.x = 0.0;
                    rateCount++;
                    break;
                }

                if (avoid_step==6 && rateCount<=30){
                    ROS_INFO("straight3\n");
                    twist.angular.z = 0.0; // 角速度
                    twist.linear.x = 0.20;
                    rateCount++;
                    break;
                }

                if (avoid_step==7 && rateCount<=30){
                    ROS_INFO("rotate4\n");
                    twist.angular.z = M_PI / 6.0; // 角速度
                    twist.linear.x = 0.0;
                    rateCount++;
                    break;
                }

                if (avoid_step==8 && rateCount<=30){
                    ROS_INFO("straight4\n");
                    twist.angular.z = 0.0; // 角速度
                    twist.linear.x = 0.20;
                    rateCount++;
                    break;
                }
                if (avoid_step==9){
                    state = RUN;
                    avoid_step = 0;
                    break;
                }
            
            case GOAL: //完全停止
                ROS_INFO("GOAL\n");
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                ROS_INFO("FINISH!\n");
                break;
        }
        twist_pub.publish(twist);
        rate.sleep();
    }
    return 0;
}
