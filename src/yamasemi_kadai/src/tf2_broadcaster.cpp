#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>


std::string goal_frame_id;
ros::Time starter_time;  //starterノードの開始時刻保持変数
bool start_time_flag = false;  //時刻を取得できたか判断するフラグ

//CSVファイルデータの読み込み関数の定義
std::vector<std::string> spilitCSVLine(std::string& input, char delimiter){
    std::istringstream stream(input);  //csvファイル内文字列を読み込むオブジェクトを作成
    std::string field;  //分割した文字列を格納する変数
    std::vector<std::string> result;  //分割した文字列を格納する配列
    while(getline(stream,field,delimiter)){  //streamから1行ずつ読み込んでdelimeterで設定した区切り文字で分割し、それをfile
        result.push_back(field);  //result配列に部分文字列(,をなくしたもの)を末尾から追加。
    }
    return result;  //全てに対して行ったresult配列を返す。※1行ごとに分かれている。
}


void startTimeCallback(const std_msgs::Time& start_time_msg){
    starter_time = start_time_msg.data;  //starter.cppから流れてきたデータを保持。
    start_time_flag = true;  //取得できたのでtrue
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "my_static_tf2_broadcaster");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    ros::Subscriber start_time_sub = nh.subscribe("/start_time",10,startTimeCallback);

    std::string file_path;
    if(!pnh.getParam("file_path",file_path)){  //launchで書いてあるファイルパスからparameterを取得する。
        ROS_ERROR_STREAM("param load error");  //もし取得できないようならエラー処理
    }

    ROS_INFO_STREAM(file_path);  //file_pathの値を出力している(あとで確認)
    std::ifstream ifs(file_path.c_str());  //ファイルパスをcの文字列に変換してファイルを開いて読み込むためのストリームを作成
    std::string line;

    std::string odom_frame_id;
    std::string goal_frame_id;
    pnh.getParam("odom_id",odom_frame_id);
    pnh.getParam("goal_id",goal_frame_id);  
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;  //BroadCasterの宣言
    geometry_msgs::TransformStamped static_transformStamped;  //メッセージ型宣言
    ros::Publisher timer_pub = nh.advertise<std_msgs::Time>("time", 10);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = odom_frame_id;
    static_transformStamped.child_frame_id = goal_frame_id;
    static_transformStamped.transform.translation.z = 0;
    ROS_INFO("Spinning until killed publishing %s to world", goal_frame_id.c_str());

    while(ros::ok() && timer_pub.getNumSubscribers() == 0);
    
    while(ros::ok() && !start_time_flag){ //start_timeが取得できなかった場合
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    
    ros::Time start = starter_time;

    while(ros::ok() && getline(ifs,line)){
        std::vector<std::string> strvec = spilitCSVLine(line,',');
        ros::Duration start_time(std::stod(strvec.at(0)));
        while(ros::Time::now()-start < start_time); //指定されたスタート時間になるまで待機
        static_transformStamped.transform.translation.x = std::stod(strvec.at(2));
        static_transformStamped.transform.translation.y = std::stod(strvec.at(3));
        tf2::Quaternion quat;
        quat.setRPY(0, 0, std::stod(strvec.at(4)));
        static_transformStamped.transform.rotation = tf2::toMsg(quat);
        static_broadcaster.sendTransform(static_transformStamped);
        ros::Duration goal_time(std::stod(strvec.at(1)));
        std_msgs::Time time_data;
        time_data.data = start + goal_time;
        timer_pub.publish(time_data);
    }
    ros::spin();
    return 0;
}


// odomからgoalの座標関係をbroadcastしているノード + CSVファイルから"スタート時間","ゴール時間","x(横)の座標","y(縦)の座標","回転量(ラジアン)"を取得。