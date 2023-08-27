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

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

//ros lib
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"

using std::placeholders::_1;

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
	
	RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Got all Params");

    laserProcessing.init(lidar_param);
	
	RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Params Initialized");
	
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                "/camera/depth/points", 
					rclcpp::SensorDataQoS(), 
					std::bind(&laserProcessingNode::velodyneHandler, this, std::placeholders::_1));
					
	RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Subscriber created");

    pubLaserCloudFiltered = create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points_filtered", 100);
    pubEdgePoints = create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100);
    pubSurfPoints = create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100); 

	RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Publishers created");

    //std::thread laser_processing_process(&laserProcessingNode::laser_processing, this);
	//RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Laser processing thread started");
}


void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudBuf.push(laserCloudMsg);
	laser_processing();
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;
int frame_count =0;
int skip_frames = 1;

void laser_processing(void){
   // while(rclcpp::ok()){
        if(!pointCloudBuf.empty()){
            //read data
             //std::lock_guard<std::mutex> lock(mutex_lock);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            rclcpp::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            //mutex_lock.unlock();

            frame_count++;
            //if(frame_count%skip_frames!=0)
            //    continue;
            //ROS_INFO("start");


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
            //ROS_INFO("total_points %d",pointcloud_in->points.size());
            sensor_msgs::msg::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
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

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    //}
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("lPNode"),"Starting Laser Processing Node Spin");
    rclcpp::spin(std::make_shared<laserProcessingNode>());
    rclcpp::shutdown();

    return 0;
}
