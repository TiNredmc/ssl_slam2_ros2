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

//ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

// tf2 lib
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationMappingClass.h"

using std::placeholders::_1;

class odomEstimationMappingNode : public rclcpp::Node 
{

public:

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;

std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudSurfBuf;

lidar::Lidar lidar_param;

std::string map_path;

std::unique_ptr<tf2_ros::TransformBroadcaster> br;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgeLaserCloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfLaserCloud;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subEdgeLaserCloud;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfLaserCloud;

odomEstimationMappingNode() : Node("odomEstimationMappingNode"){
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.05;
    get_parameter("/scan_period", scan_period); 
    get_parameter("/vertical_angle", vertical_angle); 
    get_parameter("/max_dis", max_dis);
    get_parameter("/min_dis", min_dis);
    get_parameter("/scan_line", scan_line);
    // nh.getParam("/map_resolution", map_resolution);
    // nh.getParam("/map_path", map_path);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);

    subEdgeLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                    "/laser_cloud_edge", 
						rclcpp::SensorDataQoS(), 
						std::bind(&odomEstimationMappingNode::velodyneEdgeHandler, this, std::placeholders::_1));
						
    subSurfLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                    "/laser_cloud_surf", 
	                    rclcpp::SensorDataQoS(), 
						std::bind(&odomEstimationMappingNode::velodyneSurfHandler, this, std::placeholders::_1));
	
    pubEdgeLaserCloud = create_publisher<sensor_msgs::msg::PointCloud2>("/edge_map", 10);
    pubSurfLaserCloud = create_publisher<sensor_msgs::msg::PointCloud2>("/surf_map", 10);
    pubLaserOdometry = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

	RCLCPP_INFO(rclcpp::get_logger("oEMNode"), "odom estimation and mapping node started");
}

void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
void odom_estimation(){
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            std::lock_guard<std::mutex> lock(mutex_lock);
			rclcpp::Time pointCloudSurfBuf_time = pointCloudSurfBuf.front()->header.stamp;
			rclcpp::Time pointCloudEdgeBuf_time = pointCloudEdgeBuf.front()->header.stamp;
            if(pointCloudSurfBuf_time.seconds() < pointCloudEdgeBuf_time.seconds()-0.5*lidar_param.scan_period){
                pointCloudSurfBuf.pop();
                RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(pointCloudEdgeBuf_time.seconds() < pointCloudSurfBuf_time.seconds()-0.5*lidar_param.scan_period){
                pointCloudEdgeBuf.pop();
                RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            rclcpp::Time pointcloud_time = (pointCloudEdgeBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"odom inited");
            }else{
                //ROS_WARN("current frame %d", total_frame);
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                if(total_frame%100==0)
                    RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"average odom estimation time %f ms \n \n", total_time/total_frame);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.linear());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            // publish odometry
            nav_msgs::msg::Odometry laserOdometry;
			
            laserOdometry.header.frame_id = "map"; 
            laserOdometry.child_frame_id = "base_link"; 
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
			pubLaserOdometry->publish(laserOdometry);
			//RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"odom published");
			
			//std::shared_ptr<tf2_ros::TransformBroadcaster> br;
			br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            geometry_msgs::msg::TransformStamped transform;
			transform.header.stamp = pointcloud_time;
			transform.header.frame_id = "map";
			transform.child_frame_id = "base_link";
			transform.transform.translation.x = t_current.x();
			transform.transform.translation.y = t_current.y();
			transform.transform.translation.z = t_current.z();
			transform.transform.rotation.x = q_current.x(); 
			transform.transform.rotation.y = q_current.y(); 
			transform.transform.rotation.z = q_current.z(); 
			//RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"sending transform");
            br->sendTransform(transform);
			//RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"transform sent");
			
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("oEMNode"),"Starting odom estimation and mapping Node Spin");
	auto oEMN {std::make_shared<odomEstimationMappingNode>()};
	std::thread laser_processing_process(&odomEstimationMappingNode::odom_estimation, oEMN);
    rclcpp::spin(oEMN);
	rclcpp::shutdown();

    return 0;
}
