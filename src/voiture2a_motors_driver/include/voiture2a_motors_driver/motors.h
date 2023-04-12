#ifndef MOTORS_H
#define MOTORS_H

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#define REGISTER_DATA_SIZE 44

class Motors
{
public:
    /**
     * @brief Motors
     */
    Motors(rclcpp::Node *n){
        n_ = n;
        i2c_open();

    }

    ~Motors();

    /**
     * @brief Open the I2C device
     * @return
     */
    int i2c_open();

    /**
     * @brief Send the pwm value to the thrusters \n
     * @param linear
     * @param angular
     * @return
     */
    int write_cmd_twist(const double &linear, const double &angular);

    /**
     * @brief Get data from the dspic
     */
    int get_all_data();

private:
    /**
     * @brief Send the pwm value to the thrusters \n
     * Max values \n
     *  - Stopped         127 \n
     *  - Max forward     255 \n
     *  - Max reverse     0 \n
     * @param servo
     * @param engine
     * @return 0 if success
     */
    int write_cmd(const uint8_t &servo, const uint8_t &engine) const;

    /**
     * get the version of the pic software
     * @return version
     */
    uint8_t& get_version();

public:
    /**
     *
     * @return
     */
    int get_i2c_addr() const;

    void set_i2c_addr(int i2CAddr);

    const std::string &get_i2c_periph() const;

    void set_i2c_periph(const std::string &i2c_periph);

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    int i2c_addr_ = 0x42;
    const int code_version_ = 0x03; /// Code version of the expected hardware
    uint8_t pic_code_version_=0; /// Code version read from the hardware
    std::string i2c_periph_ = "/dev/i2c-1";

    const float R1_ = 10000.0;
    const float R2_ = 10000.0;
    const float VCC_ = 3.3;
    const int nb_bits_ = 4096;

    int motor_max_pwm_ = 255;
    int motor_min_pwm_ = 0;
    int motor_stop_ = static_cast<int>(round((motor_max_pwm_-motor_min_pwm_)/2.0));

public:
    int offset_servo_ = 0;
    int offset_engine_ = 0;

public:
    std::array<uint16_t,2> pwm_value{};
    std::array<uint16_t, 18> channels{};
    uint8_t failsafe=255;
    uint8_t lost=255;
    float battery=-1;

    uint8_t cmd_servo_=0;
    uint8_t cmd_engine_=0;

};

#endif // MOTORS_H
