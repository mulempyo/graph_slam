#ifndef GRAPH_SLAM_2D_HPP
#define GRAPH_SLAM_2D_HPP

#include <ros/ros.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <vector>
#include <memory>
#include <iostream>

namespace graph_slam {

class GraphSLAM {
public:
    
    explicit GraphSLAM(const std::string& solver_type);
    int num_vertices() const;
    int num_edges() const;

    g2o::VertexSE2* add_se2_node(const Eigen::Vector3d& pose);
    g2o::EdgeSE2* add_se2_edge(g2o::VertexSE2* v1, g2o::VertexSE2* v2, const Eigen::Vector3d& relative_pose, const Eigen::Matrix3d& information_matrix);

    void detect_loop_closure(GraphSLAM& slam, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& past_scans, const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan);
    Eigen::Vector3d compute_scan_matching(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan, const pcl::PointCloud<pcl::PointXYZ>::Ptr& previous_scan);

    void optimize(int num_iterations);
    Eigen::Vector3d getOptimizedPose();

    void save(const std::string& filename);
    bool load(const std::string& filename);

    std::shared_ptr<g2o::SparseOptimizer> graph;
    std::shared_ptr<g2o::SparseOptimizer> getGraph() { return graph; }

private:


};
} // namespace graph_slam

#endif // GRAPH_SLAM_2D_HPP
