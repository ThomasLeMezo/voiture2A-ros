
#include "icm20948_driver/icm20948_node.h"
#include "icm20948_driver/FusionConverter.h"

using namespace placeholders;

ICM20948Node::ICM20948Node()
        : Node("icm20948_node"), icm20948_(){

    init_parameters();
    init_interfaces();

    if(icm20948_.init() != 0){
        RCLCPP_ERROR(this->get_logger(), "[icm20948_node] Init failed");
        exit(EXIT_FAILURE);
    }

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&ICM20948Node::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[icm20948_node] Start Ok");
}

void ICM20948Node::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));

    this->declare_parameter<int>("accel_fs_sel", icm20948_.accel_fs_sel_);
    icm20948_.accel_fs_sel_ = this->get_parameter_or("accel_fs_sel", icm20948_.accel_fs_sel_);

    this->declare_parameter<int>("gyro_fs_sel", icm20948_.gyro_fs_sel_);
    icm20948_.gyro_fs_sel_ = this->get_parameter_or("gyro_fs_sel", icm20948_.gyro_fs_sel_);


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

    this->declare_parameter<bool>("publish_raw_data", publish_raw_data_);
    publish_raw_data_ = this->get_parameter_or("publish_raw_data", publish_raw_data_);

    this->declare_parameter<bool>("publish_calibrated_data", publish_calibrated_data_);
    publish_calibrated_data_ = this->get_parameter_or("publish_calibrated_data", publish_calibrated_data_);

    this->declare_parameter<bool>("publish_rpy", publish_rpy_);
    publish_rpy_ = this->get_parameter_or("publish_rpy", publish_rpy_);

    this->declare_parameter<bool>("publish_debug_fusion", publish_debug_fusion_);
    publish_debug_fusion_ = this->get_parameter_or("publish_debug_fusion", publish_debug_fusion_);

    sample_rate_ = static_cast<unsigned int>(round(1000.0/(static_cast<double>(loop_dt_.count()))));
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
            .gyroscopeRange = static_cast<float>(icm20948_.gyro_full_scale_[icm20948_.gyro_fs_sel_]), /* replace this with actual gyroscope range in degrees/s */
            // Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this
            // feature. A value of 10 degrees is appropriate for most applications.
            .accelerationRejection = 10.0f,
            // Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature.
            // A value of 10 degrees is appropriate for most applications.
            .magneticRejection = 10.0f,
            // Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the
            // acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications.
            .recoveryTriggerPeriod = 5 * sample_rate_, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs_, &settings);
    last_time_fusion_ = steady_clock_.now();

    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Sample rate: %d", sample_rate_);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Gain: %f", settings.gain);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Gyroscope range: %f", settings.gyroscopeRange);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Acceleration rejection: %f", settings.accelerationRejection);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Magnetic rejection: %f", settings.magneticRejection);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Recovery trigger period: %d", settings.recoveryTriggerPeriod);

    // Fusion correction matrix
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Magnetometer correction matrix:");
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] %f %f %f", softIronMatrix_.element.xx, softIronMatrix_.element.xy, softIronMatrix_.element.xz);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] %f %f %f", softIronMatrix_.element.yx, softIronMatrix_.element.yy, softIronMatrix_.element.yz);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] %f %f %f", softIronMatrix_.element.zx, softIronMatrix_.element.zy, softIronMatrix_.element.zz);
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] FUSION Magnetometer correction offset:");
    RCLCPP_INFO(this->get_logger(), "[icm20948_node] %f %f %f", hardIronOffset_.axis.x, hardIronOffset_.axis.y, hardIronOffset_.axis.z);

}

void ICM20948Node::init_interfaces() {
    publisher_raw_data_ = this->create_publisher<icm20948_driver::msg::RawData>("raw_data", 1);
    publisher_calibrated_data_ = this->create_publisher<icm20948_driver::msg::RawData>("calibrated_data", 1);
    publisher_rpy_ = this->create_publisher<icm20948_driver::msg::RPY>("rpy", 1);
    publisher_debug_fusion_ = this->create_publisher<icm20948_driver::msg::DebugFusion>("debug_fusion", 1);
}

void ICM20948Node::timer_callback() {

    if(icm20948_.get_measure() != EXIT_SUCCESS)
        return;

    if(icm20948_.acc_x_ == 0. && icm20948_.acc_y_ == 0. && icm20948_.acc_z_ == 0.
        && icm20948_.gyro_x_ == 0. && icm20948_.gyro_y_ == 0. && icm20948_.gyro_z_ == 0.)
        return;

    // Publish raw data
    if(publish_raw_data_) {
        auto msg = icm20948_driver::msg::RawData();
        msg.header.stamp = this->now();
        msg.accel.x = icm20948_.acc_x_;
        msg.accel.y = icm20948_.acc_y_;
        msg.accel.z = icm20948_.acc_z_;
        msg.gyro.x = icm20948_.gyro_x_;
        msg.gyro.y = icm20948_.gyro_y_;
        msg.gyro.z = icm20948_.gyro_z_;
        msg.temp = icm20948_.temp_;
        msg.mag.x = icm20948_.mag_x_;
        msg.mag.y = icm20948_.mag_y_;
        msg.mag.z = icm20948_.mag_z_;
        publisher_raw_data_->publish(msg);
    }

    compute_ahrs();

}

void ICM20948Node::compute_ahrs() {
    // Acquire latest sensor data
    FusionVector gyroscope = {static_cast<float>(icm20948_.gyro_x_),
                              static_cast<float>(icm20948_.gyro_y_),
                              static_cast<float>(icm20948_.gyro_z_)}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {static_cast<float>(icm20948_.acc_x_),
                                  static_cast<float>(icm20948_.acc_y_),
                                  static_cast<float>(icm20948_.acc_z_)}; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {static_cast<float>(icm20948_.mag_x_),
                                 static_cast<float>(icm20948_.mag_y_),
                                 static_cast<float>(icm20948_.mag_z_)}; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment_, gyroscopeSensitivity_, gyroscopeOffset_);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment_, accelerometerSensitivity_, accelerometerOffset_);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix_, hardIronOffset_);

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset_, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    rclcpp::Time tmp_time_ = steady_clock_.now();
    const float deltaTime = (tmp_time_ - last_time_fusion_).seconds();
    last_time_fusion_ = tmp_time_;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, magnetometer, deltaTime);

    // Print algorithm outputs
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_));
    const FusionVector lin_acc = FusionAhrsGetLinearAcceleration(&ahrs_);

    // Publish message
    if(publish_rpy_) {
        auto msg = icm20948_driver::msg::RPY();
        msg.header.stamp = this->now();
        msg.roll = euler.angle.roll;
        msg.pitch = euler.angle.pitch;
        msg.yaw = euler.angle.yaw;
        msg.acceleration.x = lin_acc.axis.x;
        msg.acceleration.y = lin_acc.axis.y;
        msg.acceleration.z = lin_acc.axis.z;
        publisher_rpy_->publish(msg);
    }

    FusionAhrsInternalStates int_states = FusionAhrsGetInternalStates(&ahrs_);
    FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs_);

    // Debug Fusion
    if(publish_debug_fusion_) {
        auto msg_debug_fusion = icm20948_driver::msg::DebugFusion();
        msg_debug_fusion.header.stamp = this->now();
        msg_debug_fusion.acceleration_error = int_states.accelerationError;
        msg_debug_fusion.accelerometer_ignored = int_states.accelerometerIgnored;
        msg_debug_fusion.acceleration_recovery_trigger = int_states.accelerationRecoveryTrigger;
        msg_debug_fusion.magnetic_error = int_states.magneticError;
        msg_debug_fusion.magnetometer_ignored = int_states.magnetometerIgnored;
        msg_debug_fusion.magnetic_recovery_trigger = int_states.magneticRecoveryTrigger;
        msg_debug_fusion.initialising = flags.initialising;
        msg_debug_fusion.angular_rate_recovery = flags.angularRateRecovery;
        msg_debug_fusion.acceleration_recovery = flags.accelerationRecovery;
        msg_debug_fusion.magnetic_recovery = flags.magneticRecovery;
        msg_debug_fusion.magnetometer_data_skipped = icm20948_.mag_data_has_been_skipped_;
        msg_debug_fusion.magnetometer_limit_reached = icm20948_.mag_sensor_magnetic_overflow_;
        msg_debug_fusion.magnetometer_data_is_ready = icm20948_.mag_data_is_ready_;
        publisher_debug_fusion_->publish(msg_debug_fusion);
    }

    // Debug calibrated data
    if(publish_calibrated_data_) {
        auto msg_calibrated_data = icm20948_driver::msg::RawData();
        msg_calibrated_data.header.stamp = this->now();
        msg_calibrated_data.accel.x = accelerometer.axis.x;
        msg_calibrated_data.accel.y = accelerometer.axis.y;
        msg_calibrated_data.accel.z = accelerometer.axis.z;
        msg_calibrated_data.gyro.x = gyroscope.axis.x;
        msg_calibrated_data.gyro.y = gyroscope.axis.y;
        msg_calibrated_data.gyro.z = gyroscope.axis.z;
        msg_calibrated_data.temp = icm20948_.temp_;
        msg_calibrated_data.mag.x = magnetometer.axis.x;
        msg_calibrated_data.mag.y = magnetometer.axis.y;
        msg_calibrated_data.mag.z = magnetometer.axis.z;
        publisher_calibrated_data_->publish(msg_calibrated_data);
    }
    
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICM20948Node>());
    rclcpp::shutdown();
    return 0;
}
