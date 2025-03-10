#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <slam.h>
 
 class GraphSlamNodelet : public nodelet::Nodelet
 {
   public:
   GraphSlamNodelet()  {}
 
     ~GraphSlamNodelet() {}
   
     virtual void onInit()
     {
       NODELET_INFO_STREAM("Initialising GraphSlam nodelet...");
       sg_.reset(new graph_slam::GraphSlamNode(getNodeHandle(), getPrivateNodeHandle()));
       NODELET_INFO_STREAM("Starting live SLAM...");
       sg_->startLiveSlam();
     }
 
   private:  
     boost::shared_ptr<graph_slam::GraphSlamNode> sg_;
 };
 
 PLUGINLIB_EXPORT_CLASS(GraphSlamNodelet, nodelet::Nodelet)
