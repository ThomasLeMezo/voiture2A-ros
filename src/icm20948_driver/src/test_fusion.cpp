#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <iostream>
#include "icm20948_driver/test_fusion.h"

TestFusion::TestFusion()
        : Node("test_fusion"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[icm20948_node] Start Ok");
}

TestFusion::~TestFusion() {

}

void TestFusion::init_interfaces() {

    subscriber_raw_data_ = this->create_subscription<icm20948_driver::msg::RawData>(
            "/driver/raw_data", 1, std::bind(&TestFusion::compute_ahrs, this, std::placeholders::_1));

    publisher_calibrated_data_ = this->create_publisher<icm20948_driver::msg::RawData>("calibrated_data", 1);
    publisher_rpy_ = this->create_publisher<icm20948_driver::msg::RPY>("rpy", 1);
    publisher_debug_fusion_ = this->create_publisher<icm20948_driver::msg::DebugFusion>("debug_fusion", 1);
}

void TestFusion::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));

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
            .gyroscopeRange = static_cast<float>(250.0), /* replace this with actual gyroscope range in degrees/s */
            // Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this
            // feature. A value of 10 degrees is appropriate for most applications.
            .accelerationRejection = 3.0f,
            // Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature.
            // A value of 10 degrees is appropriate for most applications.
            .magneticRejection = 1.0f,
            // Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the
            // acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications.
            .recoveryTriggerPeriod = 1 * sample_rate_, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs_, &settings);

}

void TestFusion::compute_ahrs(const icm20948_driver::msg::RawData::SharedPtr msg) {
    if(first_msg) {
        last_time_fusion_ = msg->header.stamp;
        first_msg = false;
    }
    // Acquire latest sensor data
    FusionVector gyroscope = {static_cast<float>(msg->gyro.x),
                              static_cast<float>(msg->gyro.y),
                              static_cast<float>(msg->gyro.z)}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {static_cast<float>(msg->accel.x),
                                  static_cast<float>(msg->accel.y),
                                  static_cast<float>(msg->accel.z)}; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {static_cast<float>(0.),
                                 static_cast<float>(0.),
                                 static_cast<float>(0.)}; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment_, gyroscopeSensitivity_, gyroscopeOffset_);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment_, accelerometerSensitivity_, accelerometerOffset_);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix_, hardIronOffset_);

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset_, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    rclcpp::Time time_tmp = msg->header.stamp;
    const float deltaTime = (time_tmp - last_time_fusion_).seconds();
    last_time_fusion_ = msg->header.stamp ;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, magnetometer, deltaTime);

    // Print algorithm outputs
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_));
    const FusionVector lin_acc = FusionAhrsGetLinearAcceleration(&ahrs_);

    // Publish message
    auto msg_rpy = icm20948_driver::msg::RPY();
    msg_rpy.header.stamp = time_tmp;
    msg_rpy.roll = euler.angle.roll;
    msg_rpy.pitch = euler.angle.pitch;
    msg_rpy.yaw = euler.angle.yaw;
    msg_rpy.acceleration.x = lin_acc.axis.x;
    msg_rpy.acceleration.y = lin_acc.axis.y;
    msg_rpy.acceleration.z = lin_acc.axis.z;
    publisher_rpy_->publish(msg_rpy);

    FusionAhrsInternalStates int_states = FusionAhrsGetInternalStates(&ahrs_);
    FusionAhrsFlags flags = FusionAhrsGetFlags(&ahrs_);

    // Debug Fusion
    auto msg_debug_fusion = icm20948_driver::msg::DebugFusion();
    msg_debug_fusion.header.stamp = time_tmp;
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
    msg_debug_fusion.magnetometer_data_skipped = false;
    msg_debug_fusion.magnetometer_limit_reached = false;
    msg_debug_fusion.magnetometer_data_is_ready = false;
    publisher_debug_fusion_->publish(msg_debug_fusion);

    // Debug calibrated data
    auto msg_calibrated_data = icm20948_driver::msg::RawData();
    msg_calibrated_data.header.stamp = time_tmp;
    msg_calibrated_data.accel.x = accelerometer.axis.x;
    msg_calibrated_data.accel.y = accelerometer.axis.y;
    msg_calibrated_data.accel.z = accelerometer.axis.z;
    msg_calibrated_data.gyro.x = gyroscope.axis.x;
    msg_calibrated_data.gyro.y = gyroscope.axis.y;
    msg_calibrated_data.gyro.z = gyroscope.axis.z;
    msg_calibrated_data.temp = 0.;
    msg_calibrated_data.mag.x = magnetometer.axis.x;
    msg_calibrated_data.mag.y = magnetometer.axis.y;
    msg_calibrated_data.mag.z = magnetometer.axis.z;
    publisher_calibrated_data_->publish(msg_calibrated_data);

    RCLCPP_INFO(this->get_logger(), "R = %f, P = %f, Y = %f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestFusion>());
    rclcpp::shutdown();
    return 0;
}