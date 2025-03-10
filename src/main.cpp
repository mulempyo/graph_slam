#include <slam.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "graph_slam_node");
    graph_slam::GraphSlamNode slam_node;
    slam_node.startLiveSlam();
    ros::spin();
    return 0;
}
