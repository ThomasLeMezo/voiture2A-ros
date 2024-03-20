//
// Created by lemezoth on 29/02/24.
//

#ifndef BUILD_ANALYSIS_FUSION_NODE_H
#define BUILD_ANALYSIS_FUSION_NODE_H

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/converter.hpp>


#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "icm20948_driver/test_fusion.h"
//#include "icm20948_driver/msg/raw_data.hpp"
//#include "icm20948_driver/msg/rpy.hpp"
//#include "icm20948_driver/msg/debug_fusion.hpp"
#include "Fusion.h"

using namespace std::chrono_literals;
using namespace std;

class DataFusion{
public:
    DataFusion(const double &time,
               const double &euler_angle_roll,
               const double &euler_angle_pitch,
               const double &euler_angle_yaw,
               const double &lin_acc_axis_x,
               const double &lin_acc_axis_y,
               const double &lin_acc_axis_z,
               const double &accelerometer_axis_x,
               const double &accelerometer_axis_y,
               const double &accelerometer_axis_z,
               const double &gyroscope_axis_x,
               const double &gyroscope_axis_y,
               const double &gyroscope_axis_z,
               const double &magnetometer_axis_x,
               const double &magnetometer_axis_y,
               const double &magnetometer_axis_z,
               const double &int_states_accelerationError,
               const bool &int_states_accelerometerIgnored,
               const double &int_states_accelerationRecoveryTrigger,
               const double &int_states_magneticError,
               const bool &int_states_magnetometerIgnored,
               const double &int_states_magneticRecoveryTrigger,
               const bool &flags_initialising,
               const bool &flags_angularRateRecovery,
               const bool &flags_accelerationRecovery,
               const bool &flags_magneticRecovery
               ){
        time_ = time;
        euler_angle_roll_ = euler_angle_roll;
        euler_angle_pitch_ = euler_angle_pitch;
        euler_angle_yaw_ = euler_angle_yaw;
        lin_acc_axis_x_ = lin_acc_axis_x;
        lin_acc_axis_y_ = lin_acc_axis_y;
        lin_acc_axis_z_ = lin_acc_axis_z;
        accelerometer_axis_x_ = accelerometer_axis_x;
        accelerometer_axis_y_ = accelerometer_axis_y;
        accelerometer_axis_z_ = accelerometer_axis_z;
        gyroscope_axis_x_ = gyroscope_axis_x;
        gyroscope_axis_y_ = gyroscope_axis_y;
        gyroscope_axis_z_ = gyroscope_axis_z;
        magnetometer_axis_x_ = magnetometer_axis_x;
        magnetometer_axis_y_ = magnetometer_axis_y;
        magnetometer_axis_z_ = magnetometer_axis_z;
        int_states_accelerationError_ = int_states_accelerationError;
        int_states_accelerometerIgnored_ = int_states_accelerometerIgnored;
        int_states_accelerationRecoveryTrigger_ = int_states_accelerationRecoveryTrigger;
        int_states_magneticError_ = int_states_magneticError;
        int_states_magnetometerIgnored_ = int_states_magnetometerIgnored;
        int_states_magneticRecoveryTrigger_ = int_states_magneticRecoveryTrigger;
        flags_initialising_ = flags_initialising;
        flags_angularRateRecovery_ = flags_angularRateRecovery;
        flags_accelerationRecovery_ = flags_accelerationRecovery;
        flags_magneticRecovery_ = flags_magneticRecovery;
    }

    [[nodiscard]] static std::string serialize_variable_name() {
        // Stream the data to a string
        std::stringstream result;
        result <<
                "time" << ";" <<
                "euler_angle_roll" << ";" <<
                "euler_angle_pitch" << ";" <<
                "euler_angle_yaw" << ";" <<
                "lin_acc_axis_x" << ";" <<
                "lin_acc_axis_y" << ";" <<
                "lin_acc_axis_z" << ";" <<
                "accelerometer_axis_x" << ";" <<
                "accelerometer_axis_y" << ";" <<
                "accelerometer_axis_z" << ";" <<
                "gyroscope_axis_x" << ";" <<
                "gyroscope_axis_y" << ";" <<
                "gyroscope_axis_z" << ";" <<
                "magnetometer_axis_x" << ";" <<
                "magnetometer_axis_y" << ";" <<
                "magnetometer_axis_z" << ";" <<
                "int_states_accelerationError" << ";" <<
                "int_states_accelerometerIgnored" << ";" <<
                "int_states_accelerationRecoveryTrigger" << ";" <<
                "int_states_magneticError" << ";" <<
                "int_states_magnetometerIgnored" << ";" <<
                "int_states_magneticRecoveryTrigger" << ";" <<
                "flags_initialising" << ";" <<
                "flags_angularRateRecovery" << ";" <<
                "flags_accelerationRecovery" << ";" <<
                "flags_magneticRecovery" << ";";
        return result.str();

    }

    [[nodiscard]] std::string serialize_string() const {
        return {
                to_string(time_) + ";" +
                to_string(euler_angle_roll_) + ";" +
                to_string(euler_angle_pitch_) + ";" +
                to_string(euler_angle_yaw_) + ";" +
                to_string(lin_acc_axis_x_) + ";" +
                to_string(lin_acc_axis_y_) + ";" +
                to_string(lin_acc_axis_z_) + ";" +
                to_string(accelerometer_axis_x_) + ";" +
                to_string(accelerometer_axis_y_) + ";" +
                to_string(accelerometer_axis_z_) + ";" +
                to_string(gyroscope_axis_x_) + ";" +
                to_string(gyroscope_axis_y_) + ";" +
                to_string(gyroscope_axis_z_) + ";" +
                to_string(magnetometer_axis_x_) + ";" +
                to_string(magnetometer_axis_y_) + ";" +
                to_string(magnetometer_axis_z_) + ";" +
                to_string(int_states_accelerationError_) + ";" +
                to_string(int_states_accelerometerIgnored_) + ";" +
                to_string(int_states_accelerationRecoveryTrigger_) + ";" +
                to_string(int_states_magneticError_) + ";" +
                to_string(int_states_magnetometerIgnored_) + ";" +
                to_string(int_states_magneticRecoveryTrigger_) + ";" +
                to_string(flags_initialising_) + ";" +
                to_string(flags_angularRateRecovery_) + ";" +
                to_string(flags_accelerationRecovery_) + ";" +
                to_string(flags_magneticRecovery_) + ";"};
    }

    double time_;
    double euler_angle_roll_;
    double euler_angle_pitch_;
    double euler_angle_yaw_;
    double lin_acc_axis_x_;
    double lin_acc_axis_y_;
    double lin_acc_axis_z_;
    double accelerometer_axis_x_;
    double accelerometer_axis_y_;
    double accelerometer_axis_z_;
    double gyroscope_axis_x_;
    double gyroscope_axis_y_;
    double gyroscope_axis_z_;
    double magnetometer_axis_x_;
    double magnetometer_axis_y_;
    double magnetometer_axis_z_;
    double int_states_accelerationError_;
    bool int_states_accelerometerIgnored_;
    double int_states_accelerationRecoveryTrigger_;
    double int_states_magneticError_;
    bool int_states_magnetometerIgnored_;
    double int_states_magneticRecoveryTrigger_;
    bool flags_initialising_;
    bool flags_angularRateRecovery_;
    bool flags_accelerationRecovery_;
    bool flags_magneticRecovery_;
};

class FusionAnalysis : public rclcpp::Node {
public:
    FusionAnalysis();
    ~FusionAnalysis(){}

    std::string bag_path_ = "";
    std::string topic_raw_imu_name_ = "/driver/raw_data";
    std::string filename_export_ = "/tmp/seafoil_fusion_replay.csv";

    vector<icm20948_driver::msg::RawData> raw_data_list_;

    // Fusion
    // Rotation of 90° over z-axis
    FusionMatrix gyroscopeMisalignment_ = {0.0f, 1.0f, 0.0f,
                                           1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, -1.0f};
    FusionVector gyroscopeSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscopeOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometerMisalignment_ = gyroscopeMisalignment_;
    FusionVector accelerometerSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometerOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix magnetometerMisalignment_ = {0.0, 1.0, 0.0,
                                              -1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0 };
    FusionMatrix softIronMatrix_ = {0.9501691154747599 ,  0.04527838006278887 ,  0.011932844519038827,
                                    0.04527838006278888 ,  1.1222492815552907 ,  -0.09504905788134625,
                                    0.011932844519038824 ,  -0.09504905788134624 ,  0.9479178282010641 };

    FusionVector hardIronOffset_ = {2.371140518872212 ,  -42.80930636395084 ,  23.29334828161256};
    FusionOffset offset_{};
    FusionAhrs ahrs_{};
    FusionConvention convention_ = FusionConventionNed;
    float fusion_gain_ = 0.5f;
    unsigned int sample_rate_ = 50;
    rclcpp::Time last_time_fusion_ = rclcpp::Time(0., RCL_ROS_TIME);
    float acceleration_rejection_ = 10.0f;
    float magnetic_rejection_ = 10.0f;
    unsigned int recovery_trigger_period_ = 1;

    vector<DataFusion> df_;

    void init_parameters();

    void read_raw_data();

    void compute_ahrs();

    void export_data();
};


#endif //BUILD_ANALYSIS_FUSION_NODE_H
