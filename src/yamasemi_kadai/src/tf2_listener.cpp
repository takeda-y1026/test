#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <ros/time.h>
#include <std_msgs/Time.h>

// #include <turtlesim/Spawn.h>

bool next_flag = false;  //時刻を取得したか判定するフラグ
ros::Time goal_time;


void timeCallback(const std_msgs::Time& time_msg){  //到着した時刻をログに出力するCallback
    goal_time = time_msg.data;
    ROS_INFO("subscribe: %f",goal_time.toSec());
    next_flag = true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle nh;

    ros::Publisher base_link =
    nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);  //Publisher(geometry_msgs::Twist型メッセージをcmd_velに流す)

    ros::Subscriber timer_sub = nh.subscribe("time", 10, timeCallback);  //Subscriber
    
    tf2_ros::Buffer tfBuffer;  //TFメッセージを受信しそのメッセージからtf変換情報を取り出して保存
    tf2_ros::TransformListener tfListener(tfBuffer);  //tf変換情報からTF変換を取得する

    ros::Rate rate(10.0);


while (ros::ok())
{
    ros::NodeHandle pnh("~");
    std::string base_link_frame;
    std::string goal_frame;
    pnh.getParam("base_link_id", base_link_frame);
    pnh.getParam("goal_id", goal_frame);

    //目標時刻を取得するまで待機するループ処理。
    
    while (ros::ok() && !next_flag){ //goal_timeが取得できていない例外処理
        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = 0;  //角速度
        vel_msg.linear.x = 0;  //並進速度

        base_link.publish(vel_msg);

        ros::spinOnce();
        rate.sleep();  //無限ループを実行,spinOnceでnext_flagがtrueになればok
    }
    next_flag = false;  //上でnext_flag=trueであるため。
    

    while (ros::ok()){  //goal_timeが取得できていた場合
        ros::spinOnce();
        rate.sleep();

        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform(base_link_frame, goal_frame,
                                    ros::Time(0));  //parent_frame,child_frame
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());  //例外処理
            ros::Duration(1.0).sleep();  //１秒スリープ
            continue;
        }

        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                        transformStamped.transform.translation.x);  //角速度

        double dist = hypot(transformStamped.transform.translation.x,transformStamped.transform.translation.y);  //goalとrobotの距離計算
        vel_msg.linear.x = 1.0 * dist;  //近づくと遅くなる
        double rest_time = (goal_time-ros::Time::now()).toSec();  //到着時刻-現時刻=到達にかかる時間
        
        if(rest_time>0){
            double V = dist / rest_time;
            if(vel_msg.linear.x> V){
                vel_msg.linear.x = V;
            }
        }
        

        base_link.publish(vel_msg);
        
        if(dist < 0.015){  //1.5cmずれまで許容
            break;
        }
    }

    while (ros::ok()){  //geometryメッセージをtfに変換.姿勢をPublish
        ros::spinOnce();
        rate.sleep();
        
        geometry_msgs::Transform geo_tf;
        try{
            geo_tf = (tfBuffer.lookupTransform(base_link_frame,goal_frame,ros::Time(0))).transform;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tf2::Transform tf;
        tf2::convert(geo_tf,tf);
        double yaw = tf2::getYaw(tf.getRotation());

        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = 4.0 * yaw;
        vel_msg.linear.x = 0;

        base_link.publish(vel_msg);

        if(abs(yaw) < 5 * M_PI / 180.0){
            break;
        }
    }
}

    return 0;
};
