//
// Created by lemezoth on 29/02/24.
//

#include "icm20948_driver/fusion_analysis.h"
#include "icm20948_driver/msg/raw_data.hpp"
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <rosbag2_cpp/converter.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include "icm20948_driver/fusion_converter.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

FusionAnalysis::FusionAnalysis()
        : Node("fusion_analysis"){

    init_parameters();
    RCLCPP_INFO(this->get_logger(), "[FusionAnalysis] bag_path: %s", bag_path_.c_str());
    read_raw_data();
    compute_ahrs();
    export_data();
    exit(EXIT_SUCCESS);
}

void FusionAnalysis::init_parameters() {
    this->declare_parameter<std::string>("bag_path", bag_path_);
    bag_path_ = this->get_parameter_or("bag_path", bag_path_);

    // Set AHRS parameters
    this->declare_parameter<std::vector<double>>("gyroscopeMisalignment", FusionMatrixToStdVector(gyroscopeMisalignment_));
    gyroscopeMisalignment_ = FusionMatrixFromStdVector(this->get_parameter_or("gyroscopeMisalignment", FusionMatrixToStdVector(gyroscopeMisalignment_)));

    this->declare_parameter<std::vector<double>>("gyroscopeSensitivity", FusionVectorToStdVector(gyroscopeSensitivity_));
    gyroscopeSensitivity_ = FusionVectorFromStdVector(this->get_parameter_or("gyroscopeSensitivity", FusionVectorToStdVector(gyroscopeSensitivity_)));

    this->declare_parameter<std::vector<double>>("gyroscopeOffset", FusionVectorToStdVector(gyroscopeOffset_));
    gyroscopeOffset_ = FusionVectorFromStdVector(this->get_parameter_or("gyroscopeOffset", FusionVectorToStdVector(gyroscopeOffset_)));

    this->declare_parameter<std::vector<double>>("accelerometerMisalignment", FusionMatrixToStdVector(accelerometerMisalignment_));
    accelerometerMisalignment_ = FusionMatrixFromStdVector(this->get_parameter_or("accelerometerMisalignment", FusionMatrixToStdVector(accelerometerMisalignment_)));

    this->declare_parameter<std::vector<double>>("accelerometerSensitivity", FusionVectorToStdVector(accelerometerSensitivity_));
    accelerometerSensitivity_ = FusionVectorFromStdVector(this->get_parameter_or("accelerometerSensitivity", FusionVectorToStdVector(accelerometerSensitivity_)));

    this->declare_parameter<std::vector<double>>("accelerometerOffset", FusionVectorToStdVector(accelerometerOffset_));
    accelerometerOffset_ = FusionVectorFromStdVector(this->get_parameter_or("accelerometerOffset", FusionVectorToStdVector(accelerometerOffset_)));

    this->declare_parameter<std::vector<double>>("softIronMatrix", FusionMatrixToStdVector(softIronMatrix_));
    softIronMatrix_ = FusionMatrixFromStdVector(this->get_parameter_or("softIronMatrix", FusionMatrixToStdVector(softIronMatrix_)));

    this->declare_parameter<std::vector<double>>("hardIronOffset", FusionVectorToStdVector(hardIronOffset_));
    hardIronOffset_ = FusionVectorFromStdVector(this->get_parameter_or("hardIronOffset", FusionVectorToStdVector(hardIronOffset_)));

    this->declare_parameter<float>("fusion_gain", fusion_gain_);
    fusion_gain_ = this->get_parameter_or("fusion_gain", fusion_gain_);

    this->declare_parameter<float>("acceleration_rejection", acceleration_rejection_);
    acceleration_rejection_ = this->get_parameter_or("acceleration_rejection", acceleration_rejection_);

    this->declare_parameter<float>("magnetic_rejection", magnetic_rejection_);
    magnetic_rejection_ = this->get_parameter_or("magnetic_rejection", magnetic_rejection_);

    this->declare_parameter<int>("recovery_trigger_period", recovery_trigger_period_);
    recovery_trigger_period_ = this->get_parameter_or("recovery_trigger_period", recovery_trigger_period_);

    FusionOffsetInitialise(&offset_, sample_rate_);
    FusionAhrsInitialise(&ahrs_);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            // Earth axes convention (NWD, ENU, or NED).
            .convention = convention_,
            // Determines the influence of the gyroscope relative to other sensors. A value of zero will disable
            // initialisation and the acceleration and magnetic rejection features. A value of 0.5 is appropriate
            // for most applications.
            .gain = fusion_gain_,
            // Gyroscope range in degrees/s. Angular rate recovery will activate if the gyroscope measurement exceeds
            // 98% of this value. A value of zero will disable this feature. The value should be set to the range
            // specified in the gyroscope datasheet.
            .gyroscopeRange = 250, /* replace this with actual gyroscope range in degrees/s */
            // Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this
            // feature. A value of 10 degrees is appropriate for most applications.
            .accelerationRejection = acceleration_rejection_,
            // Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature.
            // A value of 10 degrees is appropriate for most applications.
            .magneticRejection = magnetic_rejection_,
            // Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the
            // acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications.
            .recoveryTriggerPeriod = recovery_trigger_period_ * sample_rate_, /* x seconds */
    };
    FusionAhrsSetSettings(&ahrs_, &settings);
}

void FusionAnalysis::read_raw_data(){
    // Open the rosbag
    rosbag2_storage::StorageOptions storage_options({bag_path_, "mcap"});;
    rosbag2_cpp::ConverterOptions converter_options({"cdr", "cdr"});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    // Filter to imu raw data
    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics.push_back(topic_raw_imu_name_);
    reader.set_filter(storage_filter);

    // Message deserialized
    rclcpp::Serialization<icm20948_driver::msg::RawData> serialization;
    icm20948_driver::msg::RawData msg;

    while (reader.has_next()){
        auto msg_serialized = reader.read_next();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg_serialized->serialized_data);
        serialization.deserialize_message(
                &extracted_serialized_msg, &msg);
        raw_data_list_.push_back(msg);
    }

    cout << "Load " << raw_data_list_.size() << " values" << endl;
}

void FusionAnalysis::compute_ahrs() {

    last_time_fusion_ = raw_data_list_[0].header.stamp;
    for(const auto& msg:raw_data_list_) {

        // Acquire latest sensor data
        FusionVector gyroscope = {static_cast<float>(msg.gyro.x),
                                  static_cast<float>(msg.gyro.y),
                                  static_cast<float>(msg.gyro.z)}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {static_cast<float>(msg.accel.x),
                                      static_cast<float>(msg.accel.y),
                                      static_cast<float>(msg.accel.z)}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {static_cast<float>(msg.mag.x),
                                     static_cast<float>(msg.mag.y),
                                     static_cast<float>(msg.mag.z)}; // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment_, gyroscopeSensitivity_,
                                              gyroscopeOffset_);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment_, accelerometerSensitivity_,
                                                  accelerometerOffset_);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix_, hardIronOffset_);
        magnetometer = FusionMatrixMultiplyVector(magnetometerMisalignment_, magnetometer);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset_, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error

        float deltaTime = -(last_time_fusion_ - msg.header.stamp).seconds();
        last_time_fusion_ = msg.header.stamp;

        deltaTime = 0.02;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_));
        const FusionVector lin_acc = FusionAhrsGetLinearAcceleration(&ahrs_);
        FusionAhrsInternalStates int_states = FusionAhrsGetInternalStates(&ahrs_);
        FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs_);

        df_.emplace_back(last_time_fusion_.seconds(),
                euler.angle.roll,
                euler.angle.pitch,
                euler.angle.yaw,
                lin_acc.axis.x,
                lin_acc.axis.y,
                lin_acc.axis.z,
                accelerometer.axis.x,
                accelerometer.axis.y,
                accelerometer.axis.z,
                gyroscope.axis.x,
                gyroscope.axis.y,
                gyroscope.axis.z,
                magnetometer.axis.x,
                magnetometer.axis.y,
                magnetometer.axis.z,
                int_states.accelerationError,
                int_states.accelerometerIgnored,
                int_states.accelerationRecoveryTrigger,
                int_states.magneticError,
                int_states.magnetometerIgnored,
                int_states.magneticRecoveryTrigger,
                flags.initialising,
                flags.angularRateRecovery,
                flags.accelerationRecovery,
                flags.magneticRecovery);
    }
}

void FusionAnalysis::export_data(){
    ofstream file(filename_export_);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename_export_ << endl;
        return;
    }

    file << DataFusion::serialize_variable_name() << std::endl;
    for (const auto& data : df_)
        file << data.serialize_string() << std::endl;
    file << endl;
    file.close();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionAnalysis>());
    rclcpp::shutdown();
    return 0;
}