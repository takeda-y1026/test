#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <limits> // 追加

// 入力を処理する関数
int processInput() {
    int isCorrect = 2; //default値
    std::cout << "Enter a number (1, 2, 3): ";
    int input;
    std::cin >> input;
    int correctAnswer = 1; //正解の答え 1,2,3 のどれか ここを変えて正解を変える//

    // 入力の検証
    if (std::cin.fail()) {
            std::cin.clear(); // エラー状態をクリア
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 入力バッファをクリア
            std::cout << "無効な入力です。整数を入力してください。" << std::endl << std::endl << std::endl;
    } else if (input == 1 || input == 2 || input == 3) {
        if (input == correctAnswer){ //正解のインデックスが入力されたとき
            std::cout << "正解です！！！ゲームクリア" << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
            isCorrect = 1;
        } else{
            std::cout << "不正解！ゲームオーバー" << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
            isCorrect = 0;
        }
    } else {
        std::cout << "もう一度入力してください" << std::endl << std::endl << std::endl;
    }
    
    return isCorrect;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "number_input");
    ros::NodeHandle nh;
    ros::Publisher flag_pub = nh.advertise<std_msgs::Int32>("number_input", 1000);
    ros::Rate loop_rate(10);
    std_msgs::Int32 msg;

    std::cout << "チャンスは１度きり！" << std::endl;
    while (ros::ok()){
        msg.data = processInput();
        flag_pub.publish(msg);   
        

        // if (msg.data != 2) {
        //     break; //正解or不正解のときループを抜ける
        // } else 
        // if (msg.data == 0 || msg.data == 1) {
        //     flag_pub.publish(msg);   
        //     break;
        // }
        loop_rate.sleep(); 
    } 
    
    return 0;
}