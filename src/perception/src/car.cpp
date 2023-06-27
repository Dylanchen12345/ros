#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 打印imu数据
    ROS_INFO("IMU - Orientation: [%f, %f, %f], Angular Velocity: [%f, %f, %f], Linear Acceleration: [%f, %f, %f]",
             msg->orientation.x, msg->orientation.y, msg->orientation.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 打印odom数据
    ROS_INFO("Odometry - Position: [%f, %f, %f], Orientation: [%f, %f, %f]",
             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;

    // 创建两个订阅器，分别接收imu和odom话题的消息
    ros::Subscriber imuSub = nh.subscribe("/imu/data_raw", 1, imuCallback);
    ros::Subscriber odomSub = nh.subscribe("/odom", 1, odomCallback);

    ros::spin();

    return 0;
}
