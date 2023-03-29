#include "voiture2a_motors_driver/motors.h"
#include "sys/ioctl.h"

Motors::~Motors(){
    write_cmd(MOTOR_STOP, MOTOR_STOP);
    close(file_);
}

int Motors::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[Motors_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }

    if(get_version()!=code_version_)
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Wrong PIC code version");

    usleep(100000);
    return 0;
}

int Motors::write_cmd(const uint8_t &servo, const uint8_t &engine) const{
    uint8_t data[2] = {servo, engine};

    int r = i2c_smbus_write_i2c_block_data(file_, 0x00, 2, data);
    if(r < 0)
        RCLCPP_WARN(n_->get_logger(),"[Motors_driver] I2C Bus Failure - Write cmd");
    return r;
}

int Motors::get_all_data(){
    __u8 buff[REGISTER_DATA_SIZE];
    int length = 0;
    length += i2c_smbus_read_i2c_block_data(file_, 0x00, I2C_SMBUS_BLOCK_MAX, buff);
    length += i2c_smbus_read_i2c_block_data(file_, I2C_SMBUS_BLOCK_MAX, REGISTER_DATA_SIZE-I2C_SMBUS_BLOCK_MAX, (buff+I2C_SMBUS_BLOCK_MAX));

    if (length != REGISTER_DATA_SIZE) {
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Error Reading data");
        return EXIT_FAILURE;
    }
    else{
        for(int i=0; i<2; i++)
            pwm_value[i] = buff[i*2] + (buff[i*2+1]<<8);
        for(int i=0; i<18; i++)
            channels[i] = buff[4+i*2] + (buff[5+i*2]<<8);

        failsafe = buff[0x28];
        lost = buff[0x29];
        battery = static_cast<float>(buff[0x2A] + (buff[0x2B]<<8))*R1_/(R1_+R2_)*VCC_/static_cast<float>(nb_bits_);

        return EXIT_SUCCESS;
    }
}

uint8_t& Motors::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    usleep(100);
    return pic_code_version_;
}

int Motors::get_i2c_addr() const {
    return i2c_addr_;
}

void Motors::set_i2c_addr(int i2c_addr) {
    i2c_addr_ = i2c_addr;
}

const std::string &Motors::get_i2c_periph() const {
    return i2c_periph_;
}

void Motors::set_i2c_periph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
