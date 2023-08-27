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
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

// tf2 lib
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <std_srvs/srv/empty.hpp>

#include "mapOptimizationClass.h"

using std::placeholders::_1;

class mapOptimizationNode : public rclcpp::Node 
{

public:

std::mutex mutex_lock;
std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudSurfBuf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subEdgeLaserCloud;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfLaserCloud;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;


lidar::Lidar lidar_param;
double scan_period= 0.1;
double map_resolution = 0.4;
double min_map_update_frame = 8;
double min_map_update_angle = 30;
double min_map_update_distance = 1.0;
MapOptimizationClass mapOptimization;
std::string map_path;

mapOptimizationNode() : Node("mapOptimizationNode"){
	int scan_line = 64;
    double vertical_angle = 2.0;
    double max_dis = 60.0;
    double min_dis = 2.0;

    get_parameter("/scan_period", scan_period); 
    get_parameter("/vertical_angle", vertical_angle); 
    get_parameter("/max_dis", max_dis);
    get_parameter("/min_dis", min_dis);
    get_parameter("/scan_line", scan_line);
    get_parameter("/map_resolution", map_resolution);
    get_parameter("/map_path", map_path);
    get_parameter("/min_map_update_distance", min_map_update_distance);
    get_parameter("/min_map_update_angle", min_map_update_angle);
    get_parameter("/min_map_update_frame", min_map_update_frame);
    
    mapOptimization.init(map_resolution);
    last_pose.translation().x() = 100;
	
    subEdgeLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                    "/laser_cloud_edge", 
						100, 
						std::bind(&mapOptimizationNode::velodyneEdgeHandler, this, std::placeholders::_1));
						
    subSurfLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
	                    "/laser_cloud_surf", 
						100, 
						std::bind(&mapOptimizationNode::velodyneSurfHandler, this, std::placeholders::_1));
	
    subOdometry = create_subscription<nav_msgs::msg::Odometry>(
	              "/odom", 
				  100, 
				  std::bind(&mapOptimizationNode::odomCallback, this, std::placeholders::_1));

    //map saving service
	map_save_srv_ = create_service<std_srvs::srv::Trigger>(
	                "save_map", 
					std::bind(&mapOptimizationNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    map_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map", 100);
	
    //std::thread map_optimization_process(&mapOptimizationNode::map_optimization, this);
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    odometryBuf.push(msg);
	map_optimization();
    mutex_lock.unlock();
}
void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudSurfBuf.push(laserCloudMsg);
	map_optimization();
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
	map_optimization();
    mutex_lock.unlock();
}

Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

int stamp = 0;

bool saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    
    mapOptimization.optimizeGraph(stamp, mapOptimization.getFrameNum()-1);
    stamp = mapOptimization.getFrameNum();
    mapOptimization.saveMap(map_path);
    //ROS_WARN("write feature map to folder ssl_slam2/map ...");
    res->success = true;
    res->message = "write feature map to folder ssl_slam2/map ...";
    return true;
}

int update_count = 0;
int total_frame=0;

void map_optimization(){
    while(rclcpp::ok()){
        if(!odometryBuf.empty() && !pointCloudSurfBuf.empty() && !pointCloudEdgeBuf.empty()){

            //read data
            //std::lock_guard<std::mutex> lock(mutex_lock);
			
			rclcpp::Time odometryBuf_time = odometryBuf.front()->header.stamp;
			rclcpp::Time pointCloudSurfBuf_time = pointCloudSurfBuf.front()->header.stamp;
			rclcpp::Time pointCloudEdgeBuf_time = pointCloudEdgeBuf.front()->header.stamp;
			
            if(!odometryBuf.empty() && 
			  (odometryBuf_time.seconds() < pointCloudSurfBuf_time.seconds()-0.5*lidar_param.scan_period || 
			   pointCloudSurfBuf_time.seconds() < pointCloudEdgeBuf_time.seconds()-0.5*lidar_param.scan_period)){
				   
                RCLCPP_WARN(rclcpp::get_logger("mONode"),"time stamp unaligned error and odom discarded, pls check your data --> map optimization"); 
                odometryBuf.pop();
                //mutex_lock.unlock();
                //continue;              
            }

            if(!pointCloudSurfBuf.empty() && 
			  (pointCloudSurfBuf_time.seconds() < odometryBuf_time.seconds()-0.5*lidar_param.scan_period || 
			   pointCloudSurfBuf_time.seconds() < pointCloudEdgeBuf_time.seconds()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                RCLCPP_INFO(rclcpp::get_logger("mONode"),"time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                //mutex_lock.unlock();
                //continue;  
            }

            if(!pointCloudEdgeBuf.empty() && 
			  (pointCloudEdgeBuf_time.seconds() < odometryBuf_time.seconds()-0.5*lidar_param.scan_period || 
			   pointCloudEdgeBuf_time.seconds() < pointCloudSurfBuf_time.seconds()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                RCLCPP_INFO(rclcpp::get_logger("mONode"),"time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                //mutex_lock.unlock();
                //continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            rclcpp::Time pointcloud_time = (odometryBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            odometryBuf.pop();
            //mutex_lock.unlock();
            total_frame++;   
            if(total_frame%10 == 0) 
                RCLCPP_INFO(rclcpp::get_logger("mONode"),"total_frame %d", total_frame);
            update_count++;


            Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;
            if(angular_change>90) angular_change = fabs(180 - angular_change);
            if(displacement>min_map_update_distance || angular_change>min_map_update_angle || update_count>min_map_update_frame){
                last_pose = current_pose;
                update_count=0;
                mapOptimization.addPoseToGraph(pointcloud_edge_in, pointcloud_surf_in, current_pose);
            }

            if(total_frame%30 ==0){
                sensor_msgs::msg::PointCloud2 PointsMsg;
                pcl::toROSMsg(*(mapOptimization.edgeMap)+*(mapOptimization.surfMap), PointsMsg);
                RCLCPP_INFO(rclcpp::get_logger("mONode"),"Edge Map size:%ld, Surf Map Size:%ld", (mapOptimization.edgeMap)->points.size(), (mapOptimization.surfMap)->points.size());
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub->publish(PointsMsg);
            } 

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
    rclcpp::spin(std::make_shared<mapOptimizationNode>());
	rclcpp::shutdown();

    return 0;
}
