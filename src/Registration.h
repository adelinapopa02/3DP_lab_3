#ifndef MVS_REG_REGISTRATION_H
#define MVS_REG_REGISTRATION_H
#include "open3d/Open3D.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <iostream>

struct ICPResult {
    double rmse;
    double time_ms;
    int iterations;
};

struct ICPResults {
    double rot_noise_deg;
    double trans_noise_mm;
    std::string method;
    double initial_noisy_rmse;
    ICPResult result;
};

class Registration {
public:
    Registration(std::string cloud_source_filename, std::string cloud_target_filename);
    Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target);
    void draw_registration_result();
    ICPResult execute_icp_registration(double threshold = 0.02, int max_iteration = 100, double relative_rmse = 1e-6, std::string mode = "svd");
    void execute_descriptor_registration();
    void set_transformation(Eigen::Matrix4d init_transformation);
    Eigen::Matrix4d get_transformation();
    void write_tranformation_matrix(std::string filename);
    void save_merged_cloud(std::string filename);
    double compute_rmse();
    Eigen::Matrix4d get_noisy_transformation(double rot_noise_deg_std, double trans_noise_mm);

private:
    std::tuple<std::vector<size_t>, std::vector<size_t>, double> find_closest_point(double threshold);
    Eigen::Matrix4d get_svd_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices);
    Eigen::Matrix4d get_lm_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices);
    open3d::geometry::PointCloud source_;
    open3d::geometry::PointCloud source_for_icp_;
    open3d::geometry::PointCloud target_;
    Eigen::Matrix4d transformation_ = Eigen::Matrix4d::Identity();
};

#endif // MVS_REG_REGISTRATION_H
