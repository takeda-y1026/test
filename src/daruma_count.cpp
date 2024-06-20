#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sound_play/sound_play.h>

// サウンドファイルのパスをまとめて宣言
const std::string start_sound = "/path/to/your/game_start.wav";        // ゲームスタート
const std::string daruma_sound = "/path/to/your/daruma.wav";           // だるまさんが転んだ
const std::string ten_sound = "/path/to/your/ten_seconds.wav";         // 残り10秒
const std::string five_sound = "/path/to/your/five_seconds.wav";       // 残り5秒
const std::string four_sound = "/path/to/your/four_seconds.wav";       // 残り4秒
const std::string three_sound = "/path/to/your/three_seconds.wav";     // 残り3秒
const std::string two_sound = "/path/to/your/two_seconds.wav";         // 残り2秒
const std::string one_sound = "/path/to/your/one_second.wav";          // 残り1秒
const std::string zero_sound = "/path/to/your/zero_seconds.wav";       // 残り0秒
const std::string failure_sound = "/path/to/your/game_over.wav";       // ゲームオーバー
const std::string correct_sound = "/path/to/your/correct.wav";         // ピンポン
const std::string incorrect_sound = "/path/to/your/incorrect.wav";     // ブッブー
const std::string success_sound = "/path/to/your/success.wav";         // ゲームクリア

static sound_play::SoundClient sc;

void start_voice(){
    sc.playWave(start_sound);
}
void daruma_voice(){
    sc.playWave(daruma_sound);
}
void tenSecondsLeft(){
    sc.playWave(ten_sound);
}
void fiveSecondsLeft(){
    sc.playWave(five_sound);
}
void fourSecondsLeft(){
    sc.playWave(four_sound);
}
void threeSecondsLeft(){
    sc.playWave(three_sound);
}
void twoSecondsLeft(){
    sc.playWave(two_sound);
}
void oneSecondsLeft(){
    sc.playWave(one_sound);
}
void zeroSecondsLeft(){
    sc.playWave(zero_sound);
    sc.playWave(failure_sound);
}

void timerCallback(const ros::TimerEvent& event) { // タイマーコールバック関数
    static const ros::Duration total_time(120.0); // 制限時間（2分 = 120秒）
    ros::Duration elapsed_time = event.current_real - event.last_real; // 現在の経過時間
    ros::Duration remaining_time = total_time - elapsed_time; // 残り時間

    // 残り10秒になったときに関数を呼び出す
    if (remaining_time == ros::Duration(10.0)) {
        tenSecondsLeft();
    }
    if (remaining_time == ros::Duration(5.0)) {
        fiveSecondsLeft();
    }
    if (remaining_time == ros::Duration(4.0)) {
        fourSecondsLeft();
    }
    if (remaining_time == ros::Duration(3.0)) {
        threeSecondsLeft();
    }
    if (remaining_time == ros::Duration(2.0)) {
        twoSecondsLeft();
    }
    if (remaining_time == ros::Duration(1.0)) {
        oneSecondsLeft();
    }
    if (remaining_time == ros::Duration(0.0)) {
        zeroSecondsLeft();
    }
}

void number_input_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0) //不正解
    {
        ROS_INFO("number_input incorrect, playing ブッブー.wav, ゲームオーバー.wav");
        sc.playWave(incorrect_sound);
        sc.playWave(failure_sound);
    }
    else if (msg->data == 1) //正解
    {
        ROS_INFO("number_input correct, playing ピンポン.wav, ゲームクリア.wav");
        sc.playWave(correct_sound);
        sc.playWave(success_sound);
    }
}

void detect_human_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0) //動いたとき
    {
        ROS_INFO("detect_human 0, playing ゲームオーバー.wav");
        sc.playWave(move_sound);
    }
}

void lidara_detect_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0) //動いたとき
    {
        ROS_INFO("lidar_detect 0, playing ゲームオーバー.wav");
        sc.playWave(move_sound);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "daruma_count");
    ros::NodeHandle nh;

    ros::Subscriber number_input_sub = nh.subscribe("number_input", 10, number_input_callback);
    ros::Subscriber detect_human_sub = nh.subscribe("detect_human", 10, detect_human_callback);
    ros::Subscriber lidar_detect_sub = nh.subscribe("lidar_detect", 10, lidar_detect_callback);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback); // 2分のタイマーを作成（コールバックは1秒ごとに呼び出される）
    ros::Rate loop_rate(10);

    start_voice(); //ゲームスタート

    while (ros::ok()){
        ros::spinOnce();
        daruma_voice(); //だるまさんがころんだ
        ros::Duration(1.0).sleep();
        loop_rate.sleep(); 
    } 

    return 0;
}
