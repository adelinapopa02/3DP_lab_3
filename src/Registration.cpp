#include "Registration.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

struct PointDistance {
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // This class should include an auto-differentiable cost function.
    // To rotate a point given an axis-angle rotation, use
    // the Ceres function:
    // AngleAxisRotatePoint(...) (see ceres/rotation.h)
    // Similarly to the Bundle Adjustment case initialize the struct variables with the source and the target point.
    // You have to optimize only the 6-dimensional array (rx, ry, rz, tx ,ty, tz).
    // WARNING: When dealing with the AutoDiffCostFunction template parameters,
    // pay attention to the order of the template parameters
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    PointDistance(const Eigen::Vector3d &source_point, const Eigen::Vector3d &target_point) {
        source_[0] = source_point(0);
        source_[1] = source_point(1);
        source_[2] = source_point(2);
        target_[0] = target_point(0);
        target_[1] = target_point(1);
        target_[2] = target_point(2);
    }

    template <typename T>
    bool operator()(const T *const transformation, T *residuals) const {
        T source_point[3] = {T(source_[0]), T(source_[1]), T(source_[2])};
        T rotated_point[3];
        ceres::AngleAxisRotatePoint(transformation, source_point, rotated_point);

        residuals[0] = rotated_point[0] + transformation[3] - T(target_[0]);
        residuals[1] = rotated_point[1] + transformation[4] - T(target_[1]);
        residuals[2] = rotated_point[2] + transformation[5] - T(target_[2]);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &source_point, const Eigen::Vector3d &target_point) {
        return new ceres::AutoDiffCostFunction<PointDistance, 3, 6>(
            new PointDistance(source_point, target_point));
    }

    double source_[3];
    double target_[3];
};

Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename) {
    open3d::io::ReadPointCloud(cloud_source_filename, source_);
    open3d::io::ReadPointCloud(cloud_target_filename, target_);
    Eigen::Vector3d gray_color;
    source_for_icp_ = source_;
}

Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target) {
    source_ = cloud_source;
    target_ = cloud_target;
    source_for_icp_ = source_;
}

void Registration::draw_registration_result() {
    // clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    // different color
    Eigen::Vector3d color_s;
    Eigen::Vector3d color_t;
    color_s << 1, 0.706, 0;
    color_t << 0, 0.651, 0.929;

    target_clone.PaintUniformColor(color_t);
    source_clone.PaintUniformColor(color_s);
    source_clone.Transform(transformation_);

    auto src_pointer = std::make_shared<open3d::geometry::PointCloud>(source_clone);
    auto target_pointer = std::make_shared<open3d::geometry::PointCloud>(target_clone);
    open3d::visualization::DrawGeometries({src_pointer, target_pointer});
    return;
}

ICPResult Registration::execute_icp_registration(double threshold, int max_iteration, double relative_rmse, std::string mode) {
    std::cout << "Starting ICP" << std::endl;
    ICPResult result;

    if (mode == "svd" or mode == "lm") {
        auto start = std::chrono::steady_clock::now();
        source_for_icp_.Transform(transformation_);
        double prev_rmse = std::numeric_limits<double>::infinity();
        int it;
        for (it = 0; it < max_iteration; ++it) {
            std::tuple<std::vector<size_t>, std::vector<size_t>, double> res = find_closest_point(threshold);
            auto source_indices = std::get<0>(res);
            auto target_indices = std::get<1>(res);
            double rmse = std::get<2>(res);
            std::cout << '\r' << "ICP Inlier RMSE: " << rmse << std::flush;

            if (prev_rmse - rmse < relative_rmse)
                break;
            prev_rmse = rmse;
            Eigen::Matrix4d transformation;
            if (mode == "svd")
                transformation = get_svd_icp_transformation(source_indices, target_indices);
            else if (mode == "lm")
                transformation = get_lm_icp_transformation(source_indices, target_indices);
            source_for_icp_.Transform(transformation);
            transformation_ = transformation_ * transformation;
        }
        std::cout << std::endl;
        auto end = std::chrono::steady_clock::now();
        auto time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double final_rmse = compute_rmse();
        result = {final_rmse, time_ms, it};
    } else if (mode.rfind("o3d") == 0) { // if open3d ICP method is selected
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug); // Set to Debug to get detailed information about the ICP process
        std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> transformation_estimation;
        if (mode == "o3d-p2point") {
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationPointToPoint>();
        } else if (mode == "o3d-p2plane") {
            target_.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(3, 30)); // hardcoded parameters for normal estimation, you can change them if you want
            target_.NormalizeNormals();
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationPointToPlane>();
        } else if (mode == "o3d-gen") {
            transformation_estimation = std::make_shared<open3d::pipelines::registration::TransformationEstimationForGeneralizedICP>();
        } else {
            std::cerr << "Unknown Open3D ICP mode: " << mode << std::endl;
            return {0.0, 0.0, 0};
        }
        auto start = std::chrono::steady_clock::now();
        auto reg_p2p = open3d::pipelines::registration::RegistrationICP(
            source_for_icp_,
            target_,
            threshold,
            transformation_,
            *transformation_estimation,
            open3d::pipelines::registration::ICPConvergenceCriteria(relative_rmse = relative_rmse, max_iteration = max_iteration));
        transformation_ = reg_p2p.transformation_;
        auto end = std::chrono::steady_clock::now();
        auto time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double final_rmse = compute_rmse();
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Info); // Reset verbosity level to default
        target_.normals_.clear(); // Clear normals if they were estimated for point-to-plane ICP to avoid affecting subsequent registrations
        result = {final_rmse, time_ms, 0}; // NB: unfortunately Open3D's RegistrationICP does not provide the number of iterations, so we set it to 0, you can watch the debug output to see how many iterations it performed
    } else {
        std::cerr << "Unknown ICP mode: " << mode << std::endl;
        result = {0.0, 0.0, 0};
    }

    source_for_icp_ = source_;
    return result;
}

std::tuple<std::vector<size_t>, std::vector<size_t>, double> Registration::find_closest_point(double threshold) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Use KDTreeFlann to search the closest target point for each source point.
    // Filter the correspondences based on the distance threshold.
    // Return source indices, target indices, and final RMSE.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    std::vector<size_t> source_indices;
    std::vector<size_t> target_indices;
    double mse = 0.0;
    size_t count = 0;
    const int num_src = static_cast<int>(source_for_icp_.points_.size());
    for (int i = 0; i < num_src; ++i) {
        const Eigen::Vector3d &src_pt = source_for_icp_.points_[i];
        std::vector<int> idx(1);
        std::vector<double> dist2(1);
        if (target_kd_tree.SearchKNN(src_pt, 1, idx, dist2) < 1) continue;
        if (std::sqrt(dist2[0]) > threshold) continue;
        source_indices.push_back(static_cast<size_t>(i));
        target_indices.push_back(static_cast<size_t>(idx[0]));
        mse = mse * count / (count + 1) + dist2[0] / (count + 1);
        ++count;
    }
    double rmse = (count > 0) ? std::sqrt(mse) : std::numeric_limits<double>::infinity();
    return {source_indices, target_indices, rmse};
}

Eigen::Matrix4d Registration::get_svd_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 1. Compute centroids of source and target points.
    // 2. Subtract centroids and construct matrix H.
    // 3. Use Eigen::JacobiSVD to compute rotation.
    // 4. Handle special reflection case if det(R) < 0.
    // 5. Compute translation t and build 4x4 matrix.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    const size_t n = source_indices.size();
    if (n == 0) return Eigen::Matrix4d::Identity();

    Eigen::Vector3d src_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d tgt_centroid = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < n; ++k) {
        src_centroid += source_for_icp_.points_[source_indices[k]];
        tgt_centroid += target_.points_[target_indices[k]];
    }
    src_centroid /= static_cast<double>(n);
    tgt_centroid /= static_cast<double>(n);

    Eigen::MatrixXd Src(n, 3), Tgt(n, 3);
    for (size_t k = 0; k < n; ++k) {
        Src.row(k) = (source_for_icp_.points_[source_indices[k]] - src_centroid).transpose();
        Tgt.row(k) = (target_.points_[target_indices[k]] - tgt_centroid).transpose();
    }

    Eigen::MatrixXd W = Src.transpose() * Tgt;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0.0) { V.col(2) *= -1.0; R = V * U.transpose(); }

    Eigen::Vector3d t = tgt_centroid - R * src_centroid;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

Eigen::Matrix4d Registration::get_lm_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices) {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 1. Initialize parameter vector.
    // 2. Create ceres::Problem.
    // 3. For each correspondence:
    //    - extract source/target points
    //    - add PointDistance residual block
    // 4. Call ceres::Solve(...).
    // 5. Convert axis-angle -> rotation matrix.
    // 6. Extract translation.
    // 7. Return 4x4 transformation matrix.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    const size_t n = source_indices.size();
    if (n == 0) return Eigen::Matrix4d::Identity();

    double params[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    ceres::Problem problem;
    for (size_t k = 0; k < n; ++k) {
        const Eigen::Vector3d &src_pt = source_for_icp_.points_[source_indices[k]];
        const Eigen::Vector3d &tgt_pt = target_.points_[target_indices[k]];
        problem.AddResidualBlock(PointDistance::Create(src_pt, tgt_pt), nullptr, params);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Vector3d axis_angle(params[0], params[1], params[2]);
    double angle = axis_angle.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (angle > 1e-12)
        R = Eigen::AngleAxisd(angle, axis_angle / angle).toRotationMatrix();

    Eigen::Vector3d t(params[3], params[4], params[5]);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

void Registration::execute_descriptor_registration() {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Implement a registration method based on feature descriptors.
    // - Preprocess the point clouds using Open3D functions (e.g., voxel downsampling).
    // - Detect keypoints and compute descriptors (e.g., FPFH) in both source and target clouds using Open3D methods.
    // - Match descriptors and estimate initial correspondences.
    // - Use Open3D’s RANSAC-based registration methods to reject outliers
    //   and estimate an initial rigid transformation.
    // - Do NOT use any part of ICP here; this must remain a pure
    //   descriptor-based initial alignment.
    // - Store the estimated transformation matrix in `transformation_`.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    const double voxel_size = 5.0;  
    const double normal_radius = voxel_size * 2.0;
    const int normal_max_nn = 30;
    const double fpfh_radius = voxel_size * 5.0;
    const int fpfh_max_nn = 100;
    const double ransac_dist_thresh = voxel_size * 1.5;
    const int ransac_n = 3;
    const int ransac_max_iter = 4000000;
    const double ransac_confidence = 0.999;
 
    // Downsample
    auto src_down = source_.VoxelDownSample(voxel_size);
    auto tgt_down = target_.VoxelDownSample(voxel_size);
 
    // Estimate normals
    src_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_radius, normal_max_nn));
    src_down->NormalizeNormals();
    tgt_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_radius, normal_max_nn));
    tgt_down->NormalizeNormals();
 
    // Compute FPFH
    auto src_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*src_down, open3d::geometry::KDTreeSearchParamHybrid(fpfh_radius, fpfh_max_nn));
    auto tgt_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*tgt_down, open3d::geometry::KDTreeSearchParamHybrid(fpfh_radius, fpfh_max_nn));
 
    // Correspondence checkers
    auto edge_checker = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto dist_checker = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(ransac_dist_thresh);
 
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers;
    checkers.push_back(edge_checker);
    checkers.push_back(dist_checker);
 
    // RANSAC
    auto result =
        open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            *src_down, *tgt_down,
            *src_fpfh, *tgt_fpfh,
            /*mutual_filter=*/true,
            ransac_dist_thresh,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
            ransac_n,
            checkers,
            open3d::pipelines::registration::RANSACConvergenceCriteria(ransac_max_iter, ransac_confidence));
 
    transformation_ = result.transformation_;
    std::cout << "[Descriptor] RANSAC fitness: " << result.fitness_ << "  inlier RMSE: " << result.inlier_rmse_ << std::endl;

}

void Registration::set_transformation(Eigen::Matrix4d init_transformation) {
    transformation_ = init_transformation;
}

Eigen::Matrix4d Registration::get_transformation() {
    return transformation_;
}

double Registration::compute_rmse() {
    open3d::geometry::KDTreeFlann target_kd_tree(target_);
    open3d::geometry::PointCloud source_clone = source_;
    source_clone.Transform(transformation_);
    int num_source_points = source_clone.points_.size();
    Eigen::Vector3d source_point;
    std::vector<int> idx(1);
    std::vector<double> dist2(1);
    double mse = 0.0;
    for (size_t i = 0; i < num_source_points; ++i) {
        source_point = source_clone.points_[i];
        target_kd_tree.SearchKNN(source_point, 1, idx, dist2);
        mse = mse * i / (i + 1) + dist2[0] / (i + 1);
    }
    return sqrt(mse);
}

void Registration::write_tranformation_matrix(std::string filename) {
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << transformation_;
        outfile.close();
    }
}

void Registration::save_merged_cloud(std::string filename) {
    // clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    source_clone.Transform(transformation_);
    open3d::geometry::PointCloud merged = target_clone + source_clone;
    open3d::io::WritePointCloud(filename, merged);
}

Eigen::Matrix4d Registration::get_noisy_transformation(double rot_noise_deg_std, double trans_noise_mm) {
    Eigen::Matrix4d T = get_transformation();

    static thread_local std::mt19937 gen(std::random_device{}());

    std::normal_distribution<double> rot_dist(0.0, rot_noise_deg_std);

    // centroid
    Eigen::Vector3d center_local = source_.GetCenter();

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    Eigen::Vector3d noise_rad( // rotation noise in radians
        rot_dist(gen) * M_PI / 180.0,
        rot_dist(gen) * M_PI / 180.0,
        rot_dist(gen) * M_PI / 180.0);

    double angle = noise_rad.norm();
    Eigen::Matrix3d R_noise = Eigen::Matrix3d::Identity();
    if (angle > 1e-12) {
        Eigen::Vector3d axis = noise_rad / angle;
        R_noise = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }

    Eigen::Matrix3d R_new = R * R_noise;

    Eigen::Vector3d dir = Eigen::Vector3d::Random().normalized();

    // Point clouds are stored in millimeters, so translation noise is applied directly in mm.
    Eigen::Vector3d t_noise = trans_noise_mm * dir;

    Eigen::Vector3d t_new = t + R * (Eigen::Matrix3d::Identity() - R_noise) * center_local + R * t_noise;

    Eigen::Matrix4d T_new = Eigen::Matrix4d::Identity();
    T_new.block<3, 3>(0, 0) = R_new;
    T_new.block<3, 1>(0, 3) = t_new;

    return T_new;
}