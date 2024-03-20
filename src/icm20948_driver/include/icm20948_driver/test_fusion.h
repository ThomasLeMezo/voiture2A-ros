//
// Created by lemezoth on 06/06/23.
//

#ifndef TEST_FUSION_H
#define TEST_FUSION_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "icm20948_driver/test_fusion.h"
#include "icm20948_driver/msg/raw_data.hpp"
#include "icm20948_driver/msg/rpy.hpp"
#include "icm20948_driver/msg/debug_fusion.hpp"
#include "Fusion.h"

using namespace std::chrono_literals;
using namespace std;

class TestFusion : public rclcpp::Node {
public:
    TestFusion();
    ~TestFusion();

private:

    /// Rclcpp
    std::chrono::milliseconds loop_dt_ = 20ms; /// loop dt

    /// Topics
    rclcpp::Subscription<icm20948_driver::msg::RawData>::SharedPtr subscriber_raw_data_;

    rclcpp::Publisher<icm20948_driver::msg::RawData>::SharedPtr publisher_calibrated_data_;
    rclcpp::Publisher<icm20948_driver::msg::RPY>::SharedPtr publisher_rpy_;
    rclcpp::Publisher<icm20948_driver::msg::DebugFusion>::SharedPtr publisher_debug_fusion_;

    /// Parameters

    // Fusion
    // Rotation of 90° over z-axis
    FusionMatrix gyroscopeMisalignment_ = {0.0f, 1.0f, 0.0f,
                                           -1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f};
    FusionVector gyroscopeSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscopeOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometerMisalignment_ = gyroscopeMisalignment_;
    FusionVector accelerometerSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometerOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix softIronMatrix_ = {1.0909389005666026, 0.00012587571038681422, -0.005999255022037946,
                                    0.00012587571038681278, 1.04726279047246, -0.013739953259287032,
                                    -0.005999255022037951, -0.013739953259287034, 0.8754869909015943 };

    FusionVector hardIronOffset_ = {-25.90120562612343, 81.872262863885, -73.24716722906496};
    FusionOffset offset_{};
    FusionAhrs ahrs_{};
    FusionConvention convention_ = FusionConventionNed;
    float fusion_gain_ = 0.5f;
    unsigned int sample_rate_ = 50;
    rclcpp::Time last_time_fusion_ = rclcpp::Time(0., RCL_ROS_TIME);
    bool first_msg = true;

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    void compute_ahrs(const icm20948_driver::msg::RawData::SharedPtr msg);

};

#endif //TEST_FUSION_H
