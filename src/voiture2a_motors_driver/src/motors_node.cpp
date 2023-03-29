#include "voiture2a_motors_driver/motors_node.h"

using namespace placeholders;

MotorsNode::MotorsNode()
        : Node("motors_node"), motors_(this){

    init_parameters();
    init_topics();

    motors_.i2c_open();

    timer_ = this->create_wall_timer(
            200ms, std::bind(&MotorsNode::timer_callback, this));

    motor_time_last_ = this->now();
    RCLCPP_INFO(this->get_logger(), "[Motors_node] Start Ok");
}

void MotorsNode::timer_callback() {
    float linear, angular;
    if((this->now() - motor_time_last_) > delay_stop_){
        linear = reverse_servo_?-velocity_linear_:velocity_linear_;
        angular = reverse_servo_?-velocity_angular_:velocity_angular_;
    }
    else{
        linear = 0.0;
        angular = 0.0;
    }

    auto cmd_servo = static_cast<uint8_t>(round(angular * static_cast<float>(MAX_PWM-MIN_PWM)/2.0+MOTOR_STOP));
    auto cmd_engine = static_cast<uint8_t>(round(linear * static_cast<float>(MAX_PWM-MIN_PWM)/2.0+MOTOR_STOP));

    if(cmd_servo!=cmd_servo_last_ || cmd_engine!=cmd_engine_last){
        if(motors_.write_cmd(cmd_servo, cmd_engine)==EXIT_SUCCESS){
            /// Save previous values
            cmd_servo_last_ = cmd_servo;
            cmd_engine_last = cmd_engine;

            /// Publish cmd for log
            voiture2a_motors_driver::msg::Engine msg;
            msg.servo = cmd_servo;
            msg.engine = cmd_engine;
            publisher_engine_->publish(msg);
        }
    }

    /// Get motor state
    if(motors_.get_all_data()==EXIT_SUCCESS){
        voiture2a_motors_driver::msg::MotorsState msg;
        msg.pwm_value = motors_.pwm_value;
        msg.battery = motors_.battery;
        msg.channels = motors_.channels;
        msg.failsafe = motors_.failsafe;
        msg.lost = motors_.lost;
        publisher_state_->publish(msg);
    }
}

void MotorsNode::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    this->declare_parameter<int>("delay_stop", static_cast<int>(delay_stop_.count()));
    this->declare_parameter<bool>("reverse_servo", reverse_servo_);
    this->declare_parameter<bool>("reverse_engine", reverse_engine_);

    loop_dt_ = std::chrono::milliseconds (this->get_parameter_or("loop_dt", loop_dt_.count()));
    delay_stop_ = std::chrono::milliseconds (this->get_parameter_or("delay_stop", delay_stop_.count()));
    this->get_parameter("reverse_servo", reverse_servo_);
    this->get_parameter("reverse_engine", reverse_engine_);

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", motors_.get_i2c_periph());
    this->declare_parameter<int>("i2c_address", motors_.get_i2c_addr());

    motors_.set_i2c_periph(this->get_parameter_or("i2c_periph", motors_.get_i2c_periph()));
    motors_.set_i2c_addr(this->get_parameter_or("i2c_address", motors_.get_i2c_addr()));
}

void MotorsNode::init_topics() {
    publisher_engine_ = this->create_publisher<voiture2a_motors_driver::msg::Engine>("engine", 1);
    publisher_state_ = this->create_publisher<voiture2a_motors_driver::msg::MotorsState>("state", 1);

    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorsNode::topic_velocity_callback, this, _1));
}

void MotorsNode::topic_velocity_callback(const geometry_msgs::msg::Twist &msg) {
    velocity_linear_ = std::clamp(static_cast<float>(msg.linear.x), -max_linear_velocity_, max_linear_velocity_);
    velocity_angular_ = std::clamp(static_cast<float>(msg.angular.z), -max_linear_velocity_, max_linear_velocity_);
    motor_time_last_ = this->get_clock()->now();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsNode>());
    rclcpp::shutdown();
    return 0;
}
