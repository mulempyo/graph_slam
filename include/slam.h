#ifndef GRAPH_SLAM_2D
#define GRAPH_SLAM_2D

#include <slam_algorithm.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <mutex>
#include <thread>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

namespace graph_slam{

struct ScanData {
    Eigen::Vector3d pose;
    std::vector<double> ranges;
    double timestamp;
};

class GraphSlamNode {
public:
    GraphSlamNode();
    GraphSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~GraphSlamNode();
    void init();

    void startLiveSlam();
    bool getOdomPose(Eigen::Vector3d& map_pose, const ros::Time& t);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);
    void publishTransform();
    void updateMap();
    void drawLine(int x0, int y0, int x1, int y1);

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher map_pub_, map_metadata_pub_;
    ros::ServiceServer map_service_;
    ros::Subscriber laser_sub_;
    
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    tf::Transform map_to_odom_;
    tf::TransformBroadcaster* tfB_;
    tf::Stamped<tf::Pose> centered_laser_pose_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
  
    boost::thread* transform_thread_;

    graph_slam::GraphSLAM slam_;

    nav_msgs::OccupancyGrid map_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;
    bool got_map_;
    bool got_first_scan_;
    ros::Duration map_update_interval_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> past_scans_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    int laser_count_;
    int throttle_scans_;
   
    //param
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    
    double transform_publish_period_;
    double tf_delay_;

};

}
#endif
