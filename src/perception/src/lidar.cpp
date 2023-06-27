#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr viewer;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    viewer->removePointCloud("cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_subscriber");
    ros::NodeHandle nh;

    viewer.reset(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    ros::Subscriber sub = nh.subscribe("/rslidar_points", 1, pointCloudCallback);

    ros::spin();

    return 0;
}
