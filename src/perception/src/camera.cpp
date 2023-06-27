#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 将ROS消息转换为OpenCV图像
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 在窗口中显示图像
    cv::imshow("Color Image", cv_ptr->image);
    cv::waitKey(1);
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 将ROS消息转换为OpenCV图像
    cv::Mat depth_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

    // 在窗口中显示图像
    cv::imshow("Depth Image", depth_img);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "camera_viewer");

    // 创建ROS节点句柄
    ros::NodeHandle nh;

    // 创建订阅器，订阅图像topic
    ros::Subscriber color_image_sub = nh.subscribe("/camera/color/image_raw", 10, colorImageCallback);
    ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_rect_raw", 10, depthImageCallback);

    // 进入ROS循环
    ros::spin();

    return 0;
}
