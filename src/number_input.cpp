#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <iostream>

// 入力を処理する関数
bool processInput() {
    //bool isCorrect = false;
    int isCorrect = 0;
    std::cout << "Enter a number (1, 2, 3): ";
    int input;
    std::cin >> input;
    int correctAnswer = 1; //正解の答え 1,2,3 のどれか

    // 入力の検証
    if (std::cin.fail()) {
            std::cin.clear(); // エラー状態をクリア
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 入力バッファをクリア
            std::cout << "無効な入力です。整数を入力してください。" << std::endl << std::endl << std::endl;
    } else if (input == 1 || input == 2 || input == 3) {
        if (input == correctAnswer){ //正解のインデックスが入力されたとき
            std::cout << "正解です！！！" << std::endl << std::endl << std::endl;
            isCorrect = 1;
        } else{
            std::cout << "不正解！" << std::endl << std::endl << std::endl;
            isCorrect = 2;
        }
    } else {
        std::cout << "もう一度入力してください" << std::endl << std::endl << std::endl;
    }
    
    return isCorrect;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "number_input");
    ros::NodeHandle nh;
    ros::Publisher flag_pub = nh.advertise<std_msgs::Int32>("flag", 1000);
    ros::Rate loop_rate(10);
    std_msgs::Int32 msg;

    while (ros::ok()){
        msg.data = processInput();
        flag_pub.publish(msg);   
        loop_rate.sleep(); 

        if (msg.data == true) break; //正解のときループを抜ける
    }
    
    return 0;
}
