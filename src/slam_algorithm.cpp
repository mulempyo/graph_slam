#include <ros/ros.h>
#include "slam_algorithm.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/types/slam2d/types_slam2d.h>  
#include <pcl/registration/icp.h>
#include <iostream>
#include <fstream>
#include <robust_kernel_io.h>

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace graph_slam {

    GraphSLAM::GraphSLAM(const std::string& solver_type) {
        graph = std::make_shared<g2o::SparseOptimizer>();
        g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

        std::cout << "construct solver: " << solver_type << std::endl;
        g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
        graph->setAlgorithm(solver);

        if(!graph->solver()) {
            std::cerr << "error: failed to allocate solver!" << std::endl;
            solver_factory->listSolvers(std::cerr);
            return;
        }
        std::cout << "done" << std::endl;
    }

    int GraphSLAM::num_vertices() const {
        return graph->vertices().size();
    }

    int GraphSLAM::num_edges() const {
        return graph->edges().size();
    }

    g2o::VertexSE2* GraphSLAM::add_se2_node(const Eigen::Vector3d& pose) {
        g2o::VertexSE2* vertex(new g2o::VertexSE2());
        vertex->setId(static_cast<int>(graph->vertices().size()));
        vertex->setEstimate(g2o::SE2(pose[0], pose[1], pose[2])); // x, y, theta
        graph->addVertex(vertex);
        return vertex;
    }

    g2o::EdgeSE2* GraphSLAM::add_se2_edge(g2o::VertexSE2* v1, g2o::VertexSE2* v2, const Eigen::Vector3d& relative_pose, const Eigen::Matrix3d& information_matrix) {
        g2o::EdgeSE2* edge(new g2o::EdgeSE2());
        edge->setMeasurement(g2o::SE2(relative_pose[0], relative_pose[1], relative_pose[2]));
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v1;
        edge->vertices()[1] = v2;
        graph->addEdge(edge);
        return edge;
    }

    void GraphSLAM::optimize(int num_iterations) {
        g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
        if(graph->edges().size() < 10) {
            return;
        }
        graph->initializeOptimization();
        graph->optimize(num_iterations);
    }

    Eigen::Vector3d GraphSLAM::getOptimizedPose() {
        if (graph->vertices().empty()) {
            ROS_WARN("[GraphSLAM] No vertices in graph, returning default pose.");
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    
        g2o::VertexSE2* root_vertex = dynamic_cast<g2o::VertexSE2*>(graph->vertex(0));  // 첫 번째 노드 (기준 좌표)
        if (!root_vertex) {
            ROS_WARN("[GraphSLAM] Root vertex is null, returning default pose.");
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    
        g2o::SE2 optimized_pose = root_vertex->estimate();
        return Eigen::Vector3d(optimized_pose.translation()[0], optimized_pose.translation()[1], optimized_pose.rotation().angle());
    }
    

    void GraphSLAM::save(const std::string& filename) {
        g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
        std::ofstream ofs(filename);
        graph->save(ofs);
    }

    bool GraphSLAM::load(const std::string& filename) {
        std::cout << "loading pose graph..." << std::endl;
        g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
      
        std::ifstream ifs(filename);
        if(!graph->load(ifs)) {
          return false;
        }
      
        std::cout << "nodes  : " << graph->vertices().size() << std::endl;
        std::cout << "edges  : " << graph->edges().size() << std::endl;
      
        if(!g2o::load_robust_kernels(filename + ".kernels", graph)) {
          return false;
        }
      
        return true;
      }

      Eigen::Vector3d GraphSLAM::compute_scan_matching(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& previous_scan) 
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
        if (current_scan->empty()) {
            ROS_ERROR("[ICP] Current scan is empty! Cannot perform scan matching.");
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        if (previous_scan->empty()) {
            ROS_ERROR("[ICP] Previous scan is empty! Cannot perform scan matching.");
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    
        icp.setInputSource(current_scan);
        icp.setInputTarget(previous_scan);
    
        pcl::PointCloud<pcl::PointXYZ> aligned_scan;
        icp.align(aligned_scan);
    
        if (!icp.hasConverged()) {
            ROS_ERROR("[ICP] Scan matching did not converge!");
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
    
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
        double x = transformation(0, 3);
        double y = transformation(1, 3);
        double theta = atan2(transformation(1, 0), transformation(0, 0));
    
        ROS_INFO("[ICP] Transformation computed: x=%.3f, y=%.3f, theta=%.3f", x, y, theta);
    
        return Eigen::Vector3d(x, y, theta);
    }
    

void GraphSLAM::detect_loop_closure(GraphSLAM& slam, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& past_scans, const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan) {
    for (size_t i = 0; i < past_scans.size(); i++) {
        Eigen::Vector3d relative_pose = compute_scan_matching(current_scan, past_scans[i]);

        if (relative_pose.norm() < 1.0) {  
            g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(slam.graph->vertex(i));  
            g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(slam.graph->vertex(past_scans.size()));  
            
            if (v1 && v2) { 
                slam.add_se2_edge(v1, v2, relative_pose, Eigen::Matrix3d::Identity());
            }
        }
    }
}

}  // namespace hdl_graph_slam
