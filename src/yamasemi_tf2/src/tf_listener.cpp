#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, tf::TransformListener* listener, laser_geometry::LaserProjection* projector, ros::Publisher& pub){
    // LaserScanメッセージ(msg)をPointCloudに変換
    sensor_msgs::PointCloud cloud;
    projector->projectLaser(*msg, cloud);

    // 座標変換(laserのpointcloud→odomのpointcloudへ)
    sensor_msgs::PointCloud transformed_cloud;
    try{
        ros::Time now = ros::Time::now();
    // "odom"フレームと"laser"フレームの間の座標変換が利用可能になるまで待機(ここでは１０秒待機)→超える場合は例外処理
    //指定された時間の間に最新の座標変換を受信するまでに時間がかかる場合や、バッファ内の情報が不足している場合に発生する可能性があるからそれを避けるため
        listener->waitForTransform("odom", "laser", now, ros::Duration(10.0)); 
        //cloudに格納したpointcloudをlaserからodomに座標変換してtransformed_cloudに格納
        listener->transformPointCloud("odom", cloud, transformed_cloud);
        pub.publish(transformed_cloud);
    }
    catch (tf::TransformException ex){//１０秒を超える場合には例外処理
        ROS_ERROR("%s",ex.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    ros::Publisher transLaser_pub = node.advertise<sensor_msgs::PointCloud>("transformed_pcl",10);
    tf::TransformListener listener(ros::Duration(10)); // バッファサイズを10秒に設定
    laser_geometry::LaserProjection projector;//LaserScanをpointcloudに変換するためのオブジェクト

    //_1はプレースホルダー(sensor_msgs::LaserScan::ConstPtr& msg)
    ros::Subscriber scan_sub = node.subscribe<sensor_msgs::LaserScan>("scan",20,boost::bind(scanCallback, _1, &listener, &projector, transLaser_pub));

    ros::spin();
    return 0;
}
