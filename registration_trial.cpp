#include "Registration.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char *argv[]) {
    if (argc < 4)
        std::cerr << "Usage <source pc> <target pc> <mode>";
    std::string mode = argv[3];

    Registration registration(argv[1], argv[2]);
    // registration.draw_registration_result(); // (1) Source and Target in original coordinates

    registration.execute_descriptor_registration();
    // registration.draw_registration_result(); // (2) Global Registration state
    std::cout << "Initial RMSE: " << registration.compute_rmse() << std::endl;

    if (mode != "all") {
        // uncomment to test with a single noise level
        Eigen::Matrix4d noisy_transformation = registration.get_noisy_transformation(10.0, 5.0);
        registration.set_transformation(noisy_transformation);
        registration.draw_registration_result(); // (3) Initial Noisy state
        std::cout << "Initial noisy RMSE: " << registration.compute_rmse() << std::endl;

        double diag = registration.get_diagonal();
        double threshold = std::min(registration.compute_rmse() * 2.0, diag * 0.05);
        std::cout << "ICP threshold: " << threshold << std::endl;
        int max_iterations = 100;
        double relative_rmse = 1e-4;

        ICPResult result = registration.execute_icp_registration(threshold, max_iterations, relative_rmse, mode);

        registration.draw_registration_result(); // (4) Local Refinament state

        std::cout << "Final RMSE: " << result.rmse << std::endl;
        std::cout << "Time (ms): " << result.time_ms << std::endl;
        std::cout << "Iterations: " << result.iterations << std::endl;

        registration.write_tranformation_matrix("transformation.txt");
        registration.save_merged_cloud("merged_registered_cloud.ply");
        return 0;
    }

    double rot_noise_deg_std[] = {0.0, 3.0, 5.0, 10.0}; // rotation noise in degrees
    double trans_noise_mm[] = {0.0, 1.0, 3.0, 5.0};    // translation noise in mm

    std::string icp_methods[] = {"svd", "lm", "o3d-p2point", "o3d-p2plane", "o3d-gen"};
    std::vector<ICPResults> results;

    Eigen::Matrix4d init_transformation = registration.get_transformation();

    for (double rot_noise : rot_noise_deg_std) {
        for (double trans_noise : trans_noise_mm) {
            std::cout << std::endl << "Testing with rot noise (deg): " << rot_noise << ", trans noise (mm): " << trans_noise << std::endl;

            registration.set_transformation(init_transformation); // Reset to initial transformation before adding noise

            Eigen::Matrix4d noisy_transformation = registration.get_noisy_transformation(rot_noise, trans_noise); // Generate a new noisy transformation for each noise level
            registration.set_transformation(noisy_transformation);
            // registration.draw_registration_result(); // (3) Initial Noisy state
            const double initial_noisy_rmse = registration.compute_rmse();
            std::cout << "Initial noisy RMSE: " << initial_noisy_rmse << std::endl;

            for (const std::string &method : icp_methods) {
                std::cout << std::endl << "Running ICP with method: " << method << std::endl;
                registration.set_transformation(noisy_transformation); // Reset to the same noisy transformation before each ICP method

                /////////////////////////////////////////////////////////
                // Tune these parameters
                // Can the noise level or initial rmse be good priors?
                /////////////////////////////////////////////////////////
                double diag = registration.get_diagonal();
                double threshold = std::min(registration.compute_rmse() * 2.0, diag * 0.05);
                std::cout << "ICP threshold: " << threshold << std::endl;
                int max_iterations = 100;
                double relative_rmse = 1e-4;

                ICPResult result = registration.execute_icp_registration(threshold, max_iterations, relative_rmse, method);

                // registration.draw_registration_result(); // (4) Local Refinament state
                results.push_back({rot_noise, trans_noise, method, initial_noisy_rmse, result});

                std::cout << "Final RMSE: " << result.rmse << std::endl;
                std::cout << "Time (ms): " << result.time_ms << std::endl;
                std::cout << "Iterations: " << result.iterations << std::endl;
            }
        }
    }

    const int rot_width = 10;
    const int trans_width = 10;
    const int method_width = 16;
    const int iterations_width = 15;
    const int time_width = 15;
    const int final_rmse_width = 15;
    const int initial_noisy_rmse_width = final_rmse_width;
    const int table_width = rot_width + trans_width + method_width + iterations_width + time_width + final_rmse_width + initial_noisy_rmse_width;

    std::cout << "\nICP benchmark summary\n";
    std::cout << std::right
              << std::setw(rot_width) << "rot_deg"
              << std::setw(trans_width) << "trans_mm"
              << std::setw(method_width) << "method"
              << std::setw(iterations_width) << "iterations"
              << std::setw(time_width) << "time_ms"
              << std::setw(final_rmse_width) << "final_rmse"
              << std::setw(initial_noisy_rmse_width) << "initial_noisy_rmse"
              << '\n';
    std::cout << std::string(table_width, '-') << '\n';

    std::cout << std::right << std::fixed << std::setprecision(3);
    for (const ICPResults &row : results) {
        std::cout << std::setw(rot_width) << row.rot_noise_deg
                  << std::setw(trans_width) << row.trans_noise_mm
                  << std::setw(method_width) << row.method
                  << std::setw(iterations_width) << row.result.iterations
                  << std::setw(time_width) << row.result.time_ms
                  << std::setw(final_rmse_width) << row.result.rmse
                  << std::setw(initial_noisy_rmse_width) << row.initial_noisy_rmse
                  << '\n';
    }

    std::cout << std::defaultfloat << std::endl;

    std::cout << "Note: the number of iterations for Open3D's ICP methods is not available, so it is set to 0 in the summary table. You can check the debug output to see how many iterations were performed for each method." << std::endl;
    return 0;
}
