#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
    
    ros::init(argc, argv, "urg_test");
    ros::NodeHandle nh;
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10);
    ros::Rate loop_rate(10);

    ros::spinOnce();
    int count = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        // // ロボットの正面のみのセンサの値の表示
        //int angle_center = scan.ranges.size() / 2; //384
        std::cout << "scan.ranges[384] : " << scan.ranges[384] << std::endl;

        int count = 0;
        int start_i = 0;
        int end_i = 0;
        int object1 = 0;
        int object2 = 0;
        bool continuity = false;
        for(int i=0; i<384*2; i++){
            if (count == 0 && scan.ranges[i] != INFINITY && continuity == false){
                start_i = i;
                count++;
                continuity = true;
            } else if (count == 1 && scan.ranges[i] == INFINITY && continuity == true){
                end_i = i;
                continuity = false;
            }
            object1 = (start_i + end_i) / 2;

            if (count == 1 && scan.ranges[i] != INFINITY && continuity == false){
                start_i = i;
                count++;
                continuity = true;
            } else if (count == 2 && scan.ranges[i] == INFINITY && continuity == true){
                end_i = i;
                continuity = false;
            }
            object2 = (start_i + end_i) / 2;
         
        }

        // // rvizへとscanの値をそのままPublish
        scan_pub.publish(scan);

        loop_rate.sleep();
    }
    
}
