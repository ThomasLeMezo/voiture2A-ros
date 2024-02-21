//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_ICM20948_NODE_H
#define BUILD_ICM20948_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "icm20948_driver/icm20948.h"
#include "icm20948_driver/msg/raw_data.hpp"
#include "icm20948_driver/msg/rpy.hpp"
#include "icm20948_driver/msg/debug_fusion.hpp"
#include "Fusion.h"

using namespace std::chrono_literals;
using namespace std;

class ICM20948Node : public rclcpp::Node {
public:
    ICM20948Node();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 20ms; /// loop dt

    ICM20948 icm20948_;

    /// Topics
    rclcpp::Publisher<icm20948_driver::msg::RawData>::SharedPtr publisher_raw_data_;
    rclcpp::Publisher<icm20948_driver::msg::RawData>::SharedPtr publisher_calibrated_data_;
    rclcpp::Publisher<icm20948_driver::msg::RPY>::SharedPtr publisher_rpy_;
    rclcpp::Publisher<icm20948_driver::msg::DebugFusion>::SharedPtr publisher_debug_fusion_;

    bool publish_raw_data_ = true;
    bool publish_calibrated_data_ = false;
    bool publish_rpy_ = true;
    bool publish_debug_fusion_ = false;

    /// Parameters

    // Fusion
    FusionMatrix gyroscopeMisalignment_ = {1.0f, 0.0f, 0.0f,
                                           0.0f, 1.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f};
    FusionVector gyroscopeSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscopeOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometerMisalignment_ = gyroscopeMisalignment_;
    FusionVector accelerometerSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometerOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix softIronMatrix_ = {1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0 };

    FusionVector hardIronOffset_ = {0.0, 0.0, 0.0};
    FusionOffset offset_{};
    FusionAhrs ahrs_{};
    FusionConvention convention_ = FusionConventionNed;
    float fusion_gain_ = 0.5f;
    unsigned int sample_rate_ = 50;
    rclcpp::Time last_time_fusion_ = rclcpp::Time(0., RCL_STEADY_TIME);
    rclcpp::Clock steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

    void compute_ahrs();

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    /**
     * Timer callback
     */
    void timer_callback();

};

#endif //BUILD_ICM20948_NODE_H
