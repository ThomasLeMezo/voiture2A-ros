#ifndef BUILD_THRUSTER_NODE_H
#define BUILD_THRUSTER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "voiture2a_motors_driver/msg/motors_state.hpp"
#include "voiture2a_motors_driver/msg/engine.hpp"
#include "voiture2a_motors_driver/motors.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std;

class MotorsNode : public rclcpp::Node {
public:
    MotorsNode();

private:
    /// Values
    float velocity_linear_ = 0.0;
    float velocity_angular_ = 0.0;
    rclcpp::Time motor_time_last_;

    const float max_linear_velocity_ = 1.0;
    const float max_angular_velocity_ = 1.0;
    const float min_linear_velocity_ = -1.0;
    const float min_angular_velocity_ = -1.0;

    /// I2C send
    uint8_t cmd_servo_last_ = MOTOR_STOP;
    uint8_t cmd_engine_last = MOTOR_STOP;

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt
    std::chrono::milliseconds delay_stop_ = 1000ms; /// delay to stop the motors

    /// Thrusters configuration
    bool reverse_servo_ = false;
    bool reverse_engine_ = false;

    /// I2C configuration
    Motors motors_;

    /// Topics
    rclcpp::Publisher<voiture2a_motors_driver::msg::Engine>::SharedPtr publisher_engine_;
    rclcpp::Publisher<voiture2a_motors_driver::msg::MotorsState>::SharedPtr publisher_state_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_velocity_;

    /// Functions
    void timer_callback();

    void topic_velocity_callback(const geometry_msgs::msg::Twist &msg);

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init topics to this node (publishers & subscribers)
     */
    void init_topics();

    /**
     * Invert the pwm cmd wrt MOTOR_PWM_STOP
     * @param cmd
     * @return
     */
    inline uint8_t invert_cmd(const uint8_t &cmd){
        int tmp = -(static_cast<int>(cmd) - MOTOR_STOP);
        return static_cast<uint8_t>(MOTOR_STOP + tmp);
    }
};

#endif //BUILD_THRUSTER_NODE_H
