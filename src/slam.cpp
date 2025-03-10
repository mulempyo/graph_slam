#include <slam_algorithm.h>
#include <slam.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <time.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace graph_slam{

GraphSlamNode::GraphSlamNode() : nh_(), private_nh_("~"), got_map_(false), slam_("lm_var"), transform_thread_(nullptr), scan_filter_sub_(NULL), scan_filter_(NULL) {
    map_to_odom_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    map_to_odom_.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    init();
}

GraphSlamNode::GraphSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh):
nh_(), private_nh_("~"), got_map_(false), slam_("lm_var"), transform_thread_(nullptr), scan_filter_sub_(NULL), scan_filter_(NULL)
{
    map_to_odom_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    map_to_odom_.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    init();
}

void GraphSlamNode::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    ros::Rate r(1.0 / transform_publish_period);
    while (ros::ok()) {
        publishTransform();
        r.sleep();
    }
}

GraphSlamNode::~GraphSlamNode(){
    if(transform_thread_){
        transform_thread_->join();
      }
}

void GraphSlamNode::init()
{
  got_first_scan_ = false;
  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
    
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}

void GraphSlamNode::startLiveSlam(){
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  map_service_ = nh_.advertiseService("dynamic_map", &GraphSlamNode::mapCallback, this);

  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_listener_, odom_frame_, 5);

  if (!scan_filter_) {
      ROS_ERROR("scan_filter_ is NULL!");
      return;
  }

  scan_filter_->registerCallback([this](const sensor_msgs::LaserScan::ConstPtr& msg) { 
      laserCallback(msg); 
  });

  transform_thread_ = new boost::thread(boost::bind(&GraphSlamNode::publishLoop, this, transform_publish_period_));
}

bool GraphSlamNode::getOdomPose(Eigen::Vector3d& map_pose, const ros::Time& t){
     
     centered_laser_pose_.stamp_ = t;
  
     tf::Stamped<tf::Transform> odom_pose;
     try
     {
       tf_listener_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
     }
    catch(tf::TransformException e)
     {
       ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
       return false;
     }
     double yaw = tf::getYaw(odom_pose.getRotation());

     map_pose = Eigen::Vector3d(odom_pose.getOrigin().x(),
                                odom_pose.getOrigin().y(),
                                yaw);
     return true;
}

bool GraphSlamNode::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;

  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;

  try
  {
    tf_listener_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }

  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);

  try
  {
    tf_listener_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }

  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
    return false;
  }

  double angle_center = (scan.angle_min + scan.angle_max) / 2;
  if (up.z() > 0)  
  {
    centered_laser_pose_ = tf::Stamped<tf::Pose>(
        tf::Transform(tf::createQuaternionFromRPY(0, 0, angle_center), tf::Vector3(0, 0, 0)), 
        ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else  
  {
    centered_laser_pose_ = tf::Stamped<tf::Pose>(
        tf::Transform(tf::createQuaternionFromRPY(M_PI, 0, -angle_center), tf::Vector3(0, 0, 0)), 
        ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  Eigen::Vector3d initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  return true;
}


void GraphSlamNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan = laserScanToPointCloud(scan);
  if (current_scan->empty()) {
      return;
  }

  if (!got_first_scan_) {
      if (!initMapper(*scan)) {
          return;
      }
      got_first_scan_ = true;
  }

  tf::StampedTransform odom_transform;
  try {
      tf_listener_.lookupTransform("odom", scan->header.frame_id, scan->header.stamp, odom_transform);
  } catch (tf::TransformException &e) {
      ROS_WARN("Failed to get odom transform: %s", e.what());
      return;
  }

  Eigen::Vector3d odom_pose(odom_transform.getOrigin().x(),
                            odom_transform.getOrigin().y(),
                            tf::getYaw(odom_transform.getRotation()));

  if (!getOdomPose(odom_pose, scan->header.stamp)) {
      return;
  }                        

  g2o::VertexSE2* new_node = slam_.add_se2_node(odom_pose);
  if (!new_node) {
      ROS_ERROR("Failed to add new node to graph!");
      return;
  }

  if (slam_.num_vertices() > 1 && !past_scans_.empty()) {
      g2o::VertexSE2* prev_node = dynamic_cast<g2o::VertexSE2*>(slam_.getGraph()->vertex(slam_.num_vertices() - 2));

      if (!prev_node) {
          ROS_ERROR("Previous node is NULL, skipping edge creation.");
          return;
      }

      Eigen::Vector3d relative_pose = slam_.compute_scan_matching(current_scan, past_scans_.back());
      slam_.add_se2_edge(prev_node, new_node, relative_pose, Eigen::Matrix3d::Identity());
  } else {
      ROS_WARN("[ICP] Skipping scan matching: No previous scans available.");
  }

  past_scans_.push_back(current_scan);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GraphSlamNode::laserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    double angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (std::isfinite(scan->ranges[i])) {
            pcl::PointXYZ point;
            point.x = scan->ranges[i] * cos(angle);
            point.y = scan->ranges[i] * sin(angle);
            point.z = 0.0;  

            cloud->push_back(point);
        }
        angle += scan->angle_increment;
    }
    return cloud;
}


void GraphSlamNode::updateMap() {
  boost::mutex::scoped_lock map_lock(map_mutex_);

  if (!got_map_) {
      map_.info.resolution = delta_;
      map_.header.frame_id = "map";  
      map_.info.width = static_cast<unsigned int>((xmax_ - xmin_) / delta_);
      map_.info.height = static_cast<unsigned int>((ymax_ - ymin_) / delta_);
      map_.info.origin.position.x = xmin_;
      map_.info.origin.position.y = ymin_;
      map_.info.origin.position.z = 0.0;
      map_.info.origin.orientation.w = 1.0;
      map_.data.resize(map_.info.width * map_.info.height); 
      map_.data.assign(map_.info.width * map_.info.height, -1); 
  }

  Eigen::Vector3d optimized_pose = slam_.getOptimizedPose();
  double robot_x = optimized_pose[0];
  double robot_y = optimized_pose[1];
  double robot_theta = optimized_pose[2];

  ROS_INFO("Updating map based on optimized pose: x=%f, y=%f, theta=%f", robot_x, robot_y, robot_theta);

  if (!past_scans_.empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan = past_scans_.back();

      for (const auto& point : last_scan->points) {
        
          double world_x = robot_x + point.x * cos(robot_theta) - point.y * sin(robot_theta);
          double world_y = robot_y + point.x * sin(robot_theta) + point.y * cos(robot_theta);

          int map_x = (world_x - xmin_) / delta_;
          int map_y = (world_y - ymin_) / delta_;
          int robot_map_x = (robot_x - xmin_) / delta_;
          int robot_map_y = (robot_y - ymin_) / delta_;

          drawLine(robot_map_x, robot_map_y, map_x, map_y);

          if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
            map_.data[MAP_IDX(map_.info.width, map_x, map_y)] = 100;   //obstacle
          }
    
      }
  }

  got_map_ = true;
  map_.header.stamp = ros::Time::now();
  map_pub_.publish(map_);
  map_metadata_pub_.publish(map_.info);
}

void GraphSlamNode::drawLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
      if (x0 >= 0 && x0 < map_.info.width && y0 >= 0 && y0 < map_.info.height) {
          int idx = MAP_IDX(map_.info.width, x0, y0);
          if (map_.data[idx] == -1) {  
              map_.data[idx] = 0;
          }
      }
      if (x0 == x1 && y0 == y1) break;

      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
  }
}

bool GraphSlamNode::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
  boost::mutex::scoped_lock map_lock (map_mutex_);

    if (got_map_ && map_.info.width && map_.info.height) {
        res.map = map_;
        return true;
    }
    return false;
}

void GraphSlamNode::publishTransform()
{
    map_to_odom_mutex_.lock();
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = odom_frame_;
    try {
        tf::transformTFToMsg(map_to_odom_, transform.transform);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        ROS_INFO("%s", te.what());
    }
    map_to_odom_mutex_.unlock();
}

}
