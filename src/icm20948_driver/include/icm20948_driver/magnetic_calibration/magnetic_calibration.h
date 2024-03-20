//
// Created by lemezoth on 29/02/24.
//

#ifndef BUILD_MAGNETIC_CALIBRATION_NODE_H
#define BUILD_MAGNETIC_CALIBRATION_NODE_H

#include "icm20948_driver/msg/raw_data.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

#include <vtkSmartPointer.h>
#include <vtkActor.h>

class MagneticCalibration : public rclcpp::Node {
public:
    MagneticCalibration();
    ~MagneticCalibration(){}

    std::string bag_path_ = "";
    std::string topic_raw_imu_name_ = "/driver/raw_data";
//    std::string filename_export_ = "/tmp/seafoil_fusion_replay.csv";

    double bisect_limit_width_ = 1.0;
    int bisect_limit_nb_data_ = 30;

    vector<icm20948_driver::msg::RawData> raw_data_list_;

    vector<array<double, 3>> magnetometer_data_;
    vector<array<double, 3>> magnetometer_data_regularized_;

    Vector3d center_;
    Vector3d evals_;
    Matrix3d evecs_;
    Matrix4d R_;


    void init_parameters();

    void read_raw_data();

    void regularize_data();

    void compute_ellipsoid();

    void view_data();

    vtkSmartPointer<vtkActor> generate_point_cloud(vector<array<double, 3>> &pts_data, const string &color, const double &radius = 0.1);

    vtkSmartPointer<vtkActor> generate_ellipsoid(const string &color);
};


#endif //BUILD_MAGNETIC_CALIBRATION_NODE_H
