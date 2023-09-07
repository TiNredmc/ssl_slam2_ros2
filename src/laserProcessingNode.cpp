// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

//ros lib
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"



class laserProcessingNode : public rclcpp::Node 
{

public:

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
lidar::Lidar lidar_param;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;


laserProcessingNode() : Node("laserProcessingNode"){
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    get_parameter("/scan_period", scan_period); 
    get_parameter("/vertical_angle", vertical_angle); 
    get_parameter("/max_dis", max_dis);
    get_parameter("/min_dis", min_dis);
    get_parameter("/scan_line", scan_line);
    get_parameter("/skip_frames", skip_frames);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);
	
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                "/camera/depth/points", 
			rclcpp::SensorDataQoS(), 
			std::bind(&laserProcessingNode::velodyneHandler, this, std::placeholders::_1));
					

    pubLaserCloudFiltered = create_publisher<sensor_msgs::msg::PointCloud2>("/points_filtered", 10);
    pubEdgePoints = create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 10);
    pubSurfPoints = create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 10); 

    RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Laser processing node started");
}

double total_time =0;
int total_frame=0;
int frame_count =0;
int skip_frames = 1;

void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*laserCloudMsg, *pointcloud_in);
    rclcpp::Time pointcloud_time = laserCloudMsg->header.stamp;
    std::vector<int> ind1;
    pointcloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*pointcloud_in, *pointcloud_in, ind1);
    

    //frame_count++;
    //if(frame_count%skip_frames!=0)
    //   return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZ>());          
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZ>());

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
    end = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = end - start;
    total_frame++;


    float time_temp = elapsed_seconds.count() * 1000;
    total_time+=time_temp;
    if(total_frame%500==0)
       RCLCPP_INFO(rclcpp::get_logger("lPNode"),"average laser processing time %f ms \n \n", total_time/total_frame);

    sensor_msgs::msg::PointCloud2 laserCloudFilteredMsg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());  
    *pointcloud_filtered+=*pointcloud_edge;
    *pointcloud_filtered+=*pointcloud_surf;
    pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
    laserCloudFilteredMsg.header.stamp = pointcloud_time;
    laserCloudFilteredMsg.header.frame_id = "map";
    pubLaserCloudFiltered->publish(laserCloudFilteredMsg);

    sensor_msgs::msg::PointCloud2 edgePointsMsg;
    pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
    edgePointsMsg.header.stamp = pointcloud_time;
    edgePointsMsg.header.frame_id = "base_link";
    pubEdgePoints->publish(edgePointsMsg);

    sensor_msgs::msg::PointCloud2 surfPointsMsg;
    pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
    surfPointsMsg.header.stamp = pointcloud_time;
    surfPointsMsg.header.frame_id = "base_link";
    pubSurfPoints->publish(surfPointsMsg);
    
    mutex_lock.unlock();
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Starting Laser Processing Node Spin");
    auto lPN {std::make_shared<laserProcessingNode>()};
    rclcpp::spin(lPN);
    rclcpp::shutdown();

    return 0;
}
