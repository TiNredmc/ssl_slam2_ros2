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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

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
#include <pcl/common/point_tests.h>

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

// rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subEdgeLaserCloud;
// rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfLaserCloud;
// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subEdgeLaserCloud;
message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subSurfLaserCloud;
message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> cloudodomaprox_policy;

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
	
    
    // Message filters subscribe to both PointCloud2 topics and Odometry topic
    subEdgeLaserCloud.subscribe(this, "/laser_cloud_edge");
    subSurfLaserCloud.subscribe(this, "/laser_cloud_surf"); 
    subOdometry.subscribe(this, "/odom");
	
    static message_filters::Synchronizer<cloudodomaprox_policy> syncApproximate(5, subEdgeLaserCloud, subSurfLaserCloud, subOdometry);
    syncApproximate.registerCallback(std::bind(&mapOptimizationNode::velodyneCloudOdomSyncHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	
    //map saving service
    map_save_srv_ = create_service<std_srvs::srv::Trigger>(
	                "save_map", 
			std::bind(&mapOptimizationNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    map_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);

    RCLCPP_INFO(this->get_logger(),"map Optimization node started");
}

int update_count = 0;
int total_frame = 0;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
int stamp = 0;

void velodyneCloudOdomSyncHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& laserCloudEdgeMsg, 
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& laserCloudSurfMsg,
								  const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg){

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudEdgeMsg, *pointcloud_edge_in);
	pcl::fromROSMsg(*laserCloudSurfMsg, *pointcloud_surf_in);
	Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
	current_pose.rotate(Eigen::Quaterniond(odomMsg->pose.pose.orientation.w,
	                                       odomMsg->pose.pose.orientation.x,
										   odomMsg->pose.pose.orientation.y,
										   odomMsg->pose.pose.orientation.z));  
	current_pose.pretranslate(Eigen::Vector3d(odomMsg->pose.pose.position.x,
	                                          odomMsg->pose.pose.position.y,
											  odomMsg->pose.pose.position.z));
	rclcpp::Time pointcloud_time = laserCloudEdgeMsg->header.stamp;
	total_frame++;   
	if(total_frame%10 == 0) 
		RCLCPP_INFO(this->get_logger(),"total_frame %d", total_frame);
	update_count++;

	Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
	double displacement = delta_transform.translation().squaredNorm();
	double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;
	if(angular_change>90) 
		angular_change = fabs(180 - angular_change);
	if(displacement>min_map_update_distance || angular_change>min_map_update_angle || update_count>min_map_update_frame){
		last_pose = current_pose;
		update_count=0;
		mapOptimization.addPoseToGraph(pointcloud_edge_in, pointcloud_surf_in, current_pose);
	}

	if(total_frame%5 ==0){
	RCLCPP_INFO(this->get_logger(),"Publishing Edge Map and Surf Map ...");
		sensor_msgs::msg::PointCloud2 PointsMsg;
		pcl::toROSMsg(*(mapOptimization.edgeMap)+*(mapOptimization.surfMap), PointsMsg);
		RCLCPP_INFO(this->get_logger(),"Edge Map size:%ld, Surf Map Size:%ld", (mapOptimization.edgeMap)->points.size(), (mapOptimization.surfMap)->points.size());
		PointsMsg.header.stamp = pointcloud_time;
		PointsMsg.header.frame_id = "map";
		map_pub->publish(PointsMsg);
	} 

}


bool saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    mapOptimization.optimizeGraph(stamp, mapOptimization.getFrameNum()-1);
    stamp = mapOptimization.getFrameNum();
    mapOptimization.saveMap(map_path);
    RCLCPP_WARN(this->get_logger(),"write feature map to folder ssl_slam2/map ...");
    res->success = true;
    res->message = "write feature map to folder ssl_slam2/map ...";
    return true;
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mON {std::make_shared<mapOptimizationNode>()};
    //std::thread laser_processing_process(&mapOptimizationNode::map_optimization, mON);
    rclcpp::spin(mON);
    rclcpp::shutdown();

    return 0;
}
