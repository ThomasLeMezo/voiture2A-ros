#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <vector>
#include <iostream>

#include "icm20948_driver/icm20948.h"
#include "icm20948_driver/icm20948_register.h"

ICM20948::~ICM20948() {
    if(bus_fd_ > 0)
        close(bus_fd_);
    if(bus_mag_fd_ > 0)
        close(bus_mag_fd_);
}

int ICM20948::open_i2c_mag(){
    // ************************************************
    // Open mag device
    if(i2c_bus_mag_ > 1){
        cout << "I2C bus mag number must be 0 or 1" << endl;
        return EXIT_FAILURE;
    }

    // Open I2C device
    char I2C_DEVICE_MAG[20];    // build path to I2C device
    sprintf(I2C_DEVICE_MAG, "/dev/i2c-%d", i2c_bus_mag_);
    cout << "Opening I2C device: " << I2C_DEVICE_MAG << endl;
    if ((bus_mag_fd_ = open(I2C_DEVICE_MAG, O_RDWR)) < 0) {
        perror("Error opening I2C device mag");
        return EXIT_FAILURE;
    }

    // Set I2C device address
    if (ioctl(bus_mag_fd_, I2C_SLAVE, i2c_device_mag_) < 0) {
        perror("Error setting I2C address");
        close(bus_mag_fd_);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int ICM20948::open_i2c(){
    // Open ICM device
    if(i2c_bus_ > 1){
        cout << "I2C bus number must be 0 or 1" << endl;
        return EXIT_FAILURE;
    }

    // Open I2C device
    char I2C_DEVICE[20];    // build path to I2C device
    sprintf(I2C_DEVICE, "/dev/i2c-%d", i2c_bus_);
    cout << "Opening I2C device: " << I2C_DEVICE << endl;
    if ((bus_fd_ = open(I2C_DEVICE, O_RDWR)) < 0) {
        perror("Error opening I2C device");
        return EXIT_FAILURE;
    }

    // Set I2C device address
    if (ioctl(bus_fd_, I2C_SLAVE, i2c_device_) < 0) {
        perror("Error setting I2C address");
        close(bus_fd_);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}

int ICM20948::open_spi(const unsigned int &spi_bus, const unsigned int &spi_device){
    // Check SPI bus and device number
    if(spi_bus > 1){
        cout << "SPI bus number must be 0 or 1" << endl;
        return EXIT_FAILURE;
    }
    if(spi_device > 1){
        cout << "SPI device number must be 0 or 1" << endl;
        return EXIT_FAILURE;
    }

    // Open SPI device
    char SPI_DEVICE[20];    // build path to SPI device
    sprintf(SPI_DEVICE, "/dev/spidev%d.%d", spi_bus, spi_device);
    cout << "Opening SPI device: " << SPI_DEVICE << endl;
    if ((bus_fd_ = open(SPI_DEVICE, O_RDWR)) < 0) {
        perror("Error opening SPI device");
        return EXIT_FAILURE;
    }

    // Set SPI mode and speed
    uint8_t mode = SPI_MODE_3;
    if (ioctl(bus_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Error setting SPI mode");
        close(bus_fd_);
        return EXIT_FAILURE;
    }

    if (ioctl(bus_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
        perror("Error setting SPI speed");
        close(bus_fd_);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int ICM20948::init(){
    int ret;

    if(bus_fd_ == 0){
        if(is_i2c_bus_)
            open_i2c();
        else
            open_spi(spi_bus_, spi_device_);
    }

    // ********************* //
    // Select BANK0
    select_bank(0);

    // Check WHO_AM_I
    vector<uint8_t> who_am_i = readRegister(ICM20948_BK0_WHO_AM_I);
    if(who_am_i[0] != ICM20948_WHO_AM_I){
        cout << "Error reading WHO_AM_I" << endl;
        return EXIT_FAILURE;
    }
    else{
        cout << "ICM20948 WHO_AM_I Check OK" << endl;
    }

    // Reset device
    ret = writeRegister(ICM20948_BK0_PWR_MGMT_1, imuJoinPwrMgmt1(1,0,0,0,1), false);
    if(ret != EXIT_SUCCESS){
        cout << "Error resetting device" << endl;
        return EXIT_FAILURE;
    }

    usleep(15000); // Wait for reset to complete

    // Wakeup device
    for(int i=0; i < 10; i++){
        ret = writeRegister(ICM20948_BK0_PWR_MGMT_1, imuJoinPwrMgmt1(0,0,0,0,1));
        if(ret != EXIT_SUCCESS){
            cout << "Ship might be in sleep mode wait and retry" << endl;
            if(i == 9){
                cout << "Error setting clock source" << endl;
                return EXIT_FAILURE;
            }
        }
        else{
            if(i>0)
                cout << "Retry successfully" << endl;
            break;
        }
        usleep(10000);
    }

    if(!is_i2c_bus_){
        // Set I2C_MST_EN
        ret = writeRegister(ICM20948_BK0_USER_CTRL, imuJoinUserCtrl(0,0,0,1,0,0,0), false);
        if(ret != EXIT_SUCCESS){
            cout << "Error enabling I2C master" << endl;
            return EXIT_FAILURE;
        }
    }

    // ********************* //
    // Read accelerometer compensation coefficients
    // Select BANK1
    select_bank(1);

    // Read register
    vector<uint8_t> data = readRegister(ICM20948_BK1_XA_OFFS_H, 6);
    if(data.empty()){
        cout << "Error reading accel compensation coefficients (size = " << data.size() << ")" << endl;
        cout << "Error reading accel compensation coefficients" << endl;
        return EXIT_FAILURE;
    }
    accel_x_bias_ = ((int16_t)((data[0] << 8) | data[1]));
    accel_y_bias_ = ((int16_t)((data[2] << 8) | data[3]));
    accel_z_bias_ = ((int16_t)((data[4] << 8) | data[5]));

    cout << "Accel bias : " << accel_x_bias_ << " " << accel_y_bias_ << " " << accel_z_bias_ << endl;

    // ********************* //
    // Select BANK2
    select_bank(2);

    // Gyro configuration
    ret = writeRegister(ICM20948_BK2_GYRO_SMPLRT_DIV, gyro_smplrt_div_);
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring gyros SMPLRT_DIV" << endl;
        return EXIT_FAILURE;
    }

    // Gyro configuration
    ret = writeRegister(ICM20948_BK2_GYRO_CONFIG_1, imuJoinGyroConfig1(gyro_dlpfcfg_,gyro_fs_sel_,gyro_fchoice_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring gyros 1" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_GYRO_CONFIG_2, imuJoinGyroConfig2(0,0,0,gyro_avgcfg_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring gyros 2" << endl;
        return EXIT_FAILURE;
    }

    // Accel configuration
    ret = writeRegister(ICM20948_BK2_ACCEL_SMPLRT_DIV_1, imuJoinAccelSmplrtDiv1(accel_smplrt_div_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel SMPLRT_DIV_1" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_ACCEL_SMPLRT_DIV_2, imuJoinAccelSmplrtDiv2(accel_smplrt_div_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel SMPLRT_DIV_2" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_ACCEL_INTEL_CTRL, imuJoinAccelIntelCtrl(accel_intel_en_, accel_intel_mode_int_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel INTEL_CTRL" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_ACCEL_WOM_THR, accel_intel_thr_);
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel WOM_THR" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_ACCEL_CONFIG, imuJoinAccelConfig(accel_dlpfcfg_,accel_fs_sel_,accel_fchoice_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel CONFIG" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK2_ACCEL_CONFIG_2, imuJoinAccelConfig2(0,0,0,acc_dec3Cfg_));
    if(ret != EXIT_SUCCESS){
        cout << "Error configuring accel CONFIG_2" << endl;
        return EXIT_FAILURE;
    }

    /////////////////////////////////////////////

    if(init_magnetometer() != EXIT_SUCCESS){
        cout << "Error initializing magnetometer" << endl;
        //return EXIT_FAILURE;
    }

    /////////////////////////////////////////////

    if(!is_i2c_bus_) {
        // Configure ICM to read the magnetometer
        select_bank(3);

        // Set I2C_SLV0 address to AK09916
        ret = writeRegister(ICM20948_BK3_I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C address" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C_SLV0 register to AK09916_ST1
        ret = writeRegister(ICM20948_BK3_I2C_SLV0_REG, AK09916_ST1);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C register" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C_MST_DELAY_CTRL (enable odr * 1/(1+I2C_SLV4_DLY))
        ret = writeRegister(ICM20948_BK3_I2C_MST_DELAY_CTRL, imuJoinI2cMstDelayCtrl(0, 0, 0, 0, 0, i2c_slv0_delay_en_));
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C delay control" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C_SLV4_DLY
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_CTRL, imuJoinI2cSlv4Ctrl(0, 0, 0, slv4_dly_));
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C delay" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C_SLV0 control (I2C_SLV0_EN = 1, I2C_SLV0_LENG = 8)
        ret = writeRegister(ICM20948_BK3_I2C_SLV0_CTRL, 0x89);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C control (SLVO)" << endl;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

int ICM20948::get_measure(){

    // Select BANK0
    select_bank(0);

    if(is_i2c_bus_){
        // Read data (14 bytes for acc & gyro & temp and 8 bytes for mag)
        vector<uint8_t> data = readRegister(ICM20948_BK0_ACCEL_XOUT_H, 14);
        if (data.empty()) {
//            cout << "Error reading measure data (size = " << data.size() << ")" << endl;
//            return EXIT_FAILURE;
        }
        acc_x_ = ((int16_t) ((data[0] << 8) | data[1])) * accel_sensitivity_[accel_fs_sel_]; // + accel_x_bias_;
        acc_y_ = ((int16_t) ((data[2] << 8) | data[3])) * accel_sensitivity_[accel_fs_sel_]; // + accel_y_bias_;
        acc_z_ = ((int16_t) ((data[4] << 8) | data[5])) * accel_sensitivity_[accel_fs_sel_]; // + accel_z_bias_;
        gyro_x_ = ((int16_t) ((data[6] << 8) | data[7])) * gyro_sensitivity_[gyro_fs_sel_];
        gyro_y_ = ((int16_t) ((data[8] << 8) | data[9])) * gyro_sensitivity_[gyro_fs_sel_];
        gyro_z_ = ((int16_t) ((data[10] << 8) | data[11])) * gyro_sensitivity_[gyro_fs_sel_];
        temp_ = (float) ((int16_t) ((data[12] << 8) | data[13]) / 333.87 + 21.0);

        // Read magnetometer data
        vector<uint8_t> data_mag = read_i2c_AK09916_data(AK09916_ST1, 9);
        if (data_mag.empty()) {
//            cout << "Error reading AK09916 data (size = " << data_mag.size() << ")" << endl;
//            return EXIT_FAILURE;
        }
        else {
            mag_x_ = ((int16_t) (data_mag[1] | (data_mag[2] << 8))) * mag_sensitivity_;
            mag_y_ = ((int16_t) (data_mag[3] | (data_mag[4] << 8))) * mag_sensitivity_;
            mag_z_ = ((int16_t) (data_mag[5] | (data_mag[6] << 8))) * mag_sensitivity_;
            mag_sensor_magnetic_overflow_ = (data_mag[7] & 0x08) >> 3;
            mag_data_is_ready_ = (data_mag[0] & 0x01);
            mag_data_has_been_skipped_ = (data_mag[0] & 0x02) >> 1;
        }


    }
    else {
        // Read data (14 bytes for acc & gyro & temp and 8 bytes for mag)
        vector<uint8_t> data = readRegister(ICM20948_BK0_ACCEL_XOUT_H, 23);
        if (data.empty()) {
            cout << "Error reading measure data (size = " << data.size() << ")" << endl;
            return EXIT_FAILURE;
        }

        acc_x_ = ((int16_t) ((data[0] << 8) | data[1])) * accel_sensitivity_[accel_fs_sel_]; // + accel_x_bias_;
        acc_y_ = ((int16_t) ((data[2] << 8) | data[3])) * accel_sensitivity_[accel_fs_sel_]; // + accel_y_bias_;
        acc_z_ = ((int16_t) ((data[4] << 8) | data[5])) * accel_sensitivity_[accel_fs_sel_]; // + accel_z_bias_;
        gyro_x_ = ((int16_t) ((data[6] << 8) | data[7])) * gyro_sensitivity_[gyro_fs_sel_];
        gyro_y_ = ((int16_t) ((data[8] << 8) | data[9])) * gyro_sensitivity_[gyro_fs_sel_];
        gyro_z_ = ((int16_t) ((data[10] << 8) | data[11])) * gyro_sensitivity_[gyro_fs_sel_];
        temp_ = (float) ((int16_t) ((data[12] << 8) | data[13]) / 333.87 + 21.0);

        mag_x_ = ((int16_t) (data[15] | (data[16] << 8))) * mag_sensitivity_;
        mag_y_ = ((int16_t) (data[17] | (data[18] << 8))) * mag_sensitivity_;
        mag_z_ = ((int16_t) (data[19] | (data[20] << 8))) * mag_sensitivity_;
        mag_sensor_magnetic_overflow_ = (data[21] & 0x08) >> 3;
        mag_data_is_ready_ = (data[14] & 0x01);
        mag_data_has_been_skipped_ = (data[14] & 0x02) >> 1;
    }

    return EXIT_SUCCESS;
}

int ICM20948::reset_i2c(){
    int ret = writeRegister(ICM20948_BK0_USER_CTRL, imuJoinUserCtrl(0,0,1,i2c_if_dis_,0,0,1), false); // Autoreset of last 3 bits
    if(ret != EXIT_SUCCESS){
        cout << "Error disabling I2C" << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int ICM20948::write_i2c_AK09916_byte(const uint8_t &reg, const uint8_t &data){
    if(is_i2c_bus_){
        int ret = i2c_smbus_write_byte_data(bus_mag_fd_, reg, data);
        if(ret<0){
            cout << "Error writing I2C register (AK09916)" << (int)reg << endl;
            return EXIT_FAILURE;
        }
        else
            return EXIT_SUCCESS;
    }
    else {
        int ret;
        // Slect BANK3
        ret = select_bank(3);

        // Set I2C Addr (Write = 0x00)
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_ADDR, AK09916_ADDRESS | 0x00);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C address (write_i2c_AK09916_byte)" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C REG
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_REG, reg);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C register" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C Data
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_DO, data);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C data" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C Control (I2C_SLV0_REG_DIS = 1)
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_CTRL, imuJoinI2cSlv4Ctrl(1, 1, 0, slv4_dly_), false);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C control (SLV4)" << endl;
            return EXIT_FAILURE;
        }


        int max_cycles = 1000;
        int count = 0;
        while (++count < max_cycles) {
            select_bank(0);
            vector<uint8_t> i2c_mst_status = readRegister(ICM20948_BK0_I2C_MST_STATUS);
            if (i2c_mst_status[0] & (1 << 6) || count > max_cycles)
                break;
            usleep(1000);
        }

        if (count > max_cycles) {
            cout << "Error write_i2c_AK09916_byte I2C_MST_STATUS" << endl;
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }
}

int ICM20948::reset_i2c_slv4(){
    // Slect BANK3
    int ret = writeRegister(ICM20948_BK0_REG_BANK_SEL, imuJoinBankId(3));
    if(ret != EXIT_SUCCESS){
        cout << "Error selecting BANK3" << endl;
        return EXIT_FAILURE;
    }

    // Set I2C Addr (Write = 0x00)
    ret = writeRegister(ICM20948_BK3_I2C_SLV4_ADDR, 0x00);
    if(ret != EXIT_SUCCESS){
        cout << "Error writing I2C address" << endl;
        return EXIT_FAILURE;
    }

    // Set I2C REG
    ret = writeRegister(ICM20948_BK3_I2C_SLV4_REG, 0x00);
    if(ret != EXIT_SUCCESS){
        cout << "Error writing I2C register" << endl;
        return EXIT_FAILURE;
    }

    ret = writeRegister(ICM20948_BK3_I2C_SLV4_DO, 0x00);
    if(ret != EXIT_SUCCESS){
        cout << "Error writing I2C data" << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

std::vector<uint8_t> ICM20948::read_i2c_AK09916_data(const uint8_t &reg, const uint32_t &len) {
    std::vector<uint8_t> dataVector;
    dataVector.resize(len);
    int nb_read = i2c_smbus_read_i2c_block_data(bus_mag_fd_, reg, len, dataVector.data());
    if (nb_read != len) {
        cout << "Error reading I2C register (AK09916, data) " << (int) reg  << " " << nb_read << " " << len << endl;
        return {};
    } else
        return dataVector;
}

int ICM20948::read_i2c_AK09916_byte(const uint8_t &reg, uint8_t &data){

    if(is_i2c_bus_){
        data = i2c_smbus_read_byte_data(bus_mag_fd_, reg);
        if(data<0){
            cout << "Error reading I2C register (AK09916)" << (int)reg << endl;
            return EXIT_FAILURE;
        }
        else
            return EXIT_SUCCESS;
    }
    else {

        // Select BANK3
        select_bank(3);

        // Set I2C Addr (Read = 0x80)
        int ret = writeRegister(ICM20948_BK3_I2C_SLV4_ADDR, AK09916_ADDRESS | 0x80);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C address (read_i2c_AK09916_byte)" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C REG
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_REG, reg);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C register" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C Control (I2C_SLV4_EN = 1)
        ret = writeRegister(ICM20948_BK3_I2C_SLV4_CTRL, imuJoinI2cSlv4Ctrl(1, 1, 0, slv4_dly_), false);
        if (ret != EXIT_SUCCESS) {
            cout << "Error writing I2C control" << endl;
            return EXIT_FAILURE;
        }
        usleep(1000);

        int max_cycles = 1000;
        int count = 0;
        while (++count < max_cycles) {
            select_bank(0);
            vector<uint8_t> i2c_mst_status = readRegister(ICM20948_BK0_I2C_MST_STATUS);
            if (i2c_mst_status[0] & (1 << 6) || count > max_cycles)
                break;
            usleep(1000);
        }

        if (count >= max_cycles) {
            cout << "Error read_i2c_AK09916_byte I2C_MST_STATUS" << endl;
            return EXIT_FAILURE;
        }

        // Read data
        vector<uint8_t> rx = readRegister(ICM20948_BK3_I2C_SLV4_DI, 1);
        if (rx.empty()) {
            cout << "Error reading AK09916 data (size = " << rx.size() << ")" << endl;
            return EXIT_FAILURE;
        }
        data = rx[0];
    }

    return EXIT_SUCCESS;
}

std::vector<uint8_t> ICM20948::readRegister(const uint8_t &reg, const uint32_t &len) {
    if(is_i2c_bus_) {
        std::vector<uint8_t> dataVector;
        dataVector.resize(len);
        if (i2c_smbus_read_i2c_block_data(bus_fd_, reg, len, dataVector.data()) != len) {
            cout << "Error reading I2C register (icm) " << (int) reg << endl;
            return {};
        } else
            return dataVector;
    }
    else {
        vector<uint8_t> rx(len + 1, 0);
        vector<uint8_t> tx(len + 1, 0);

        tx[0] = reg | 0x80;

        struct spi_ioc_transfer mesg = {
                .tx_buf = (unsigned long) tx.data(),
                .rx_buf = (unsigned long) rx.data(),
                .len = len + 1,
                .speed_hz = speed_,
                .delay_usecs = delay_usecs_,
                .bits_per_word = 8,
                //.cs_change = 1
        };

        if (ioctl(bus_fd_, SPI_IOC_MESSAGE(1), &mesg) < 0) {
            perror("Error reading SPI register");
            close(bus_fd_);
            return {};
        }
        usleep(25);

        return {rx.begin() + 1, rx.end()};
    }
}

int ICM20948::writeRegister(const uint8_t &reg, const uint8_t &data, bool check_write){
    int ret = writeRegister(reg, vector<uint8_t>(1,data));

    if(check_write){
        vector<uint8_t> read_data = readRegister(reg, 1);
        if(read_data.empty() || read_data[0] != data){
            cout << "Error writing register (Register : " << (int)reg <<
                 ", Write = " << (int)data <<
                 ", Read = "<< (int)read_data[0] <<
                 ")" << endl;
            return EXIT_FAILURE;
        }
    }

    return ret;
}

int ICM20948::writeRegister(const uint8_t &reg, const std::vector<uint8_t> &data) {

    if(is_i2c_bus_){
        if(i2c_smbus_write_i2c_block_data(bus_fd_, reg, data.size(), data.data()) < 0){
            cout << "Error writing I2C register " << (int)reg << endl;
            return EXIT_FAILURE;
        }
        else
            return EXIT_SUCCESS;
    }
    else {
        vector<uint8_t> tx;
        tx.reserve(data.size() + 1);
        vector<uint8_t> rx(data.size() + 1, 0);

        tx.push_back(reg);
        copy(data.begin(), data.end(), back_inserter(tx));

        struct spi_ioc_transfer mesg = {
                .tx_buf = (unsigned long) tx.data(),
                .rx_buf = (unsigned long) rx.data(),
                .len = static_cast<__u32>(data.size() + 1),
                .speed_hz = speed_,
                .delay_usecs = delay_usecs_,
                .bits_per_word = 8
        };

        if (ioctl(bus_fd_, SPI_IOC_MESSAGE(1), &mesg) < 0) {
            perror("Error writing SPI register");
            close(bus_fd_);
            return EXIT_FAILURE;
        }
        usleep(25);
    }

    return EXIT_SUCCESS;
}

uint8_t ICM20948::imuJoinUserCtrl(uint8_t dmpEn,
                                  uint8_t fifoEn,
                                  uint8_t i2cMstEn,
                                  uint8_t i2cIfDis,
                                  uint8_t dmpRst,
                                  uint8_t sramRst,
                                  uint8_t i2cMstRst) {

    return ((i2cMstRst & 1) << 1) +
           ((sramRst & 1) << 2) +
           ((dmpRst & 1) << 3) +
           ((i2cIfDis & 1) << 4) +
           ((i2cMstEn & 1) << 5) +
           ((fifoEn & 1) << 6) +
           ((dmpEn & 1) << 7);
}

uint8_t ICM20948::imuJoinLpConfig(uint8_t i2cMstCycle,uint8_t accelCycle,uint8_t gyroCycle) {
    return ((gyroCycle & 1) << 4) + ((accelCycle & 1) << 5) + ((i2cMstCycle & 1) << 6);
}

uint8_t ICM20948::imuJoinPwrMgmt1(uint8_t deviceRst, uint8_t sleep, uint8_t lpEn, uint8_t tempDis, uint8_t clkSel) {
    return ((deviceRst & 1) << 7) +
           ((sleep & 1) << 6) +
           ((lpEn & 1) << 5) +
           ((tempDis & 1) << 3) +
           ((clkSel & 7) << 0);
}

uint8_t ICM20948::imuJoinPwrMgmt2(uint8_t disableAccel,uint8_t disableGyro) {
    return (disableGyro & 7) + ((disableAccel & 7) << 3);
}

uint8_t ICM20948::joinGyroSmplrtDiv(uint8_t gyroSmplrtDiv) {
    return gyroSmplrtDiv & 255;
}
uint8_t ICM20948::imuJoinGyroConfig1(uint8_t gyroDlpfCfg,uint8_t gyroFsSel,uint8_t gyroFchoice) {
    return (gyroFchoice & 1) + ((gyroFsSel &  3) << 1) + ((gyroDlpfCfg & 7) << 3);
}

uint8_t ICM20948::imuJoinGyroConfig2(uint8_t xGyroCten,uint8_t yGyroCten,uint8_t zGyroCten,uint8_t gyroAvgcfg) {
    return (gyroAvgcfg & 3) + ((zGyroCten & 1) << 3) + ((yGyroCten & 1) << 4) + ((xGyroCten & 1) << 5);
}

uint8_t ICM20948::imuJoinAccelSmplrtDiv1(uint16_t accelSmplrtDiv) {
    return (accelSmplrtDiv >> 8) & 0xf;
}

uint8_t ICM20948::imuJoinAccelSmplrtDiv2(uint16_t accelSmplrtDiv) {
    return accelSmplrtDiv & 0xff;
}

uint8_t ICM20948::imuJoinAccelIntelCtrl(uint8_t accelIntelEn,uint8_t accelIntelModeInt) {
    return (accelIntelModeInt & 1) + ((accelIntelEn & 1) << 1);
}

uint8_t ICM20948::imuJoinAccelConfig(uint8_t accelDlpfcfg,uint8_t accelFsSel,uint8_t accelFchoice) {
    return (accelFchoice & 1) + ((accelFsSel & 3) << 1) + ((accelDlpfcfg & 7) << 3);
}

uint8_t ICM20948::imuJoinAccelConfig2(uint8_t axStEnReg,uint8_t ayStEnReg,uint8_t azStEnReg,uint8_t dec3Cfg) {
    return (dec3Cfg & 3) + ((azStEnReg & 1) << 2) + ((ayStEnReg & 1) << 3) + ((axStEnReg & 1) << 4);
}

uint8_t ICM20948::imuJoinBankId(uint8_t bank_id) {
    return (bank_id & 3) << 4;
}

uint8_t ICM20948::imuJoinMstCtrl(uint8_t multMstEn, uint8_t i2cMstPNsr, uint8_t i2cMstClk) {
    return (i2cMstClk & 15) + ((i2cMstPNsr & 1) << 4) + ((multMstEn & 1) << 7);
}

uint8_t ICM20948::imuJoinIntEnable(uint8_t regWofEn, uint8_t womIntEn, uint8_t pllRdyEn, uint8_t dmpInt1En, uint8_t i2cMstIntEn){
    return (i2cMstIntEn & 1) + ((dmpInt1En & 1) << 1) + ((pllRdyEn & 1) << 2) + ((womIntEn & 1) << 3) + ((regWofEn & 1) << 7);

}

uint8_t ICM20948::imuJoinI2cMstDelayCtrl(uint8_t delayEsShadow, uint8_t i2cSlv4DelayEn, uint8_t i2cSlv3DelayEn,
                                         uint8_t i2cSlv2DelayEn, uint8_t i2cSlv1DelayEn, uint8_t i2cSlv0DelayEn) {
    return (i2cSlv0DelayEn & 1) +
           ((i2cSlv1DelayEn & 1) << 1) +
           ((i2cSlv2DelayEn & 1) << 2) +
           ((i2cSlv3DelayEn & 1) << 3) +
           ((i2cSlv4DelayEn & 1) << 4) +
           ((delayEsShadow & 1) << 7);
}

uint8_t ICM20948::imuJoinI2cSlv4Ctrl(uint8_t i2cSlv4En, uint8_t i2cSlv4IntEn, uint8_t i2cSlv4RegDis,
                                     uint8_t i2cSlv4Dly) {
    return (i2cSlv4Dly & 15) +
           ((i2cSlv4RegDis & 1) << 5) +
           ((i2cSlv4IntEn & 1) << 6) +
           ((i2cSlv4En & 1) << 7);
}

uint8_t ICM20948::imuJoinIntPinCfg(uint8_t int1Actl, uint8_t int1Open, uint8_t int1LatchEn, uint8_t int1Anyrd2Clear,
                                   uint8_t int1FsyncLvl, uint8_t int1FsyncEn, uint8_t int1BypassEn) {
    return ((int1BypassEn & 1) << 1) +
           ((int1FsyncEn & 1) << 2) +
           ((int1FsyncLvl & 1) << 3) +
           ((int1Anyrd2Clear & 1) << 4) +
           ((int1LatchEn & 1) << 5) +
           ((int1Open & 1) << 6) +
           ((int1Actl & 1) << 7);
}

// Dump of all register of BANK3 of ICM20948
void ICM20948::dump_bank(const int &bank_id){
    // Select BANK3
    int ret = writeRegister(ICM20948_BK0_REG_BANK_SEL, imuJoinBankId(bank_id));
    if(ret != EXIT_SUCCESS){
        cout << "Error selecting BANK3" << endl;
        return;
    }

    // Read data (14 bytes for acc & gyro & temp and 7 bytes for mag)
    vector<uint8_t> data = readRegister(0, 128);
    if(data.empty()){
        cout << "Error reading accel data (size = " << data.size() << ")" << endl;
        cout << "Error reading accel data" << endl;
        return;
    }

    cout << "Dump of BANK" << (int)bank_id << endl;
    for(int i=0; i<128; i++){
        cout << "0x" << hex << (int)data[i] << " ";
        if(i%16 == 15)
            cout << endl;
    }
    cout << endl;
}

void ICM20948::dump_icm20948(){
    for(int i=0; i<4; i++){
        dump_bank(i);
    }
}

void ICM20948::dump_ak09916(){
    for(int i=0; i<52; i++){
        uint8_t data;
        read_i2c_AK09916_byte(i, data);
        cout << "register " << i << " : " << (int)data << endl;
    }
}

int ICM20948::select_bank(const uint8_t &bank_id){
    if(current_bank_ == bank_id)
        return EXIT_SUCCESS;
    // Select BANK3
    int ret = writeRegister(ICM20948_BK0_REG_BANK_SEL, imuJoinBankId(bank_id));
    if(ret != EXIT_SUCCESS){
        cout << "Error selecting BANK" << bank_id << endl;
    }
    else
        current_bank_ = bank_id;
    return ret;
}

int ICM20948::i2c_master_reset(){
    int ret;
    select_bank(0);

    // Read USER CTRL register
    vector<uint8_t> user_ctrl = readRegister(ICM20948_BK0_USER_CTRL);
    if(user_ctrl.empty()){
        cout << "Error reading USER_CTRL" << endl;
        return EXIT_FAILURE;
    }

    user_ctrl[0] |= 0x01 << 1; // Set I2C_MST_RST bit

    // Reset I2C Master
    ret = writeRegister(ICM20948_BK0_USER_CTRL, user_ctrl[0], false); // Autoreset of last 3 bits
    if(ret != EXIT_SUCCESS){
        cout << "Error disabling I2C" << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool ICM20948::test_who_i_am_mag(){
    int ret;
    uint8_t who_am_i_mag = 0;
    ret = read_i2c_AK09916_byte(AK09916_WHO_AM_I_REG, who_am_i_mag);
    if (ret != EXIT_SUCCESS) {
        cout << "Error reading AK09916 WHO_AM_I" << endl;
        return false;
    }
    if (who_am_i_mag != AK09916_WHO_AM_I) {
        cout << "Error AK09916 WHO_AM_I (Read = " << (int) who_am_i_mag << ")" << endl;
        return false;
    }
    else {
        cout << "AK09916 WHO_AM_I Check OK" << endl;
        return true;
    }
}

int ICM20948::set_mag_mode(const AK09916_mode_e &mode){
    int ret;

    // read CNTL2
    uint8_t cntl2 = 0;
    ret = read_i2c_AK09916_byte(AK09916_CNTL2, cntl2);
    if (ret != EXIT_SUCCESS) {
        cout << "Error reading AK09916 CNTL2" << endl;
        return EXIT_FAILURE;
    }

    if(cntl2 != AK09916_mode_power_down) {
        // Configure ICM20948 for reading the magnetometer
        ret &= write_i2c_AK09916_byte(AK09916_CNTL2, AK09916_mode_power_down); // Then set continuous mode 100Hz
        if (ret != EXIT_SUCCESS) {
            cout << "Error setting AK09916 CNTL2 (power down)" << endl;
            return EXIT_FAILURE;
        }

        usleep(10000);
    }

    // Set power up with frequency = mag_mode_
    ret &= write_i2c_AK09916_byte(AK09916_CNTL2, (uint8_t)mode); // Then set continuous mode 100Hz
    if(ret != EXIT_SUCCESS){
        cout << "Error setting AK09916 CNTL2" << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int ICM20948::init_magnetometer(){
// ************************************************************************************** //
    // Init magnetometer
    int ret;

    if(is_i2c_bus_) {
        // Set bypass mode off
        select_bank(0);
        ret = writeRegister(ICM20948_BK0_INT_PIN_CFG, imuJoinIntPinCfg(0, 0, 0, 0, 0, 0, 1));
        if(ret != EXIT_SUCCESS){
            cout << "Error setting I2C bypass mode" << endl;
            return EXIT_FAILURE;
        }

        open_i2c_mag();
    }
    else{
        // Set bypass mode off
        select_bank(0);
        ret = writeRegister(ICM20948_BK0_INT_PIN_CFG, imuJoinIntPinCfg(0, 0, 0, 0, 0, 0, 0));
        if(ret != EXIT_SUCCESS){
            cout << "Error setting I2C bypass mode" << endl;
            return EXIT_FAILURE;
        }

        // Set I2C Master clock to 400kHz
        select_bank(3);
        ret = writeRegister(ICM20948_BK3_I2C_MST_CTRL, imuJoinMstCtrl(0, 1, i2c_frequency_clk_)); // Test restart/stop ?
        if (ret != EXIT_SUCCESS) {
            cout << "Error setting I2C Master clock" << endl;
            return EXIT_FAILURE;
        }

        // Enable I2C
        select_bank(0);
        ret = writeRegister(ICM20948_BK0_USER_CTRL, imuJoinUserCtrl(0, 0, 1, i2c_if_dis_, 0, 0, 0),
                            false); // Autoreset of last 3 bits
        if (ret != EXIT_SUCCESS) {
            cout << "Error disabling I2C" << endl;
            return EXIT_FAILURE;
        }

        // Enable I2C interrupt
        ret = writeRegister(ICM20948_BK0_INT_ENABLE, imuJoinIntEnable(0, 0, 0, 0, 1));
        if (ret != EXIT_SUCCESS) {
            cout << "Error enabling I2C interrupt" << endl;
            return EXIT_FAILURE;
        }
    }

    /////////////////////////////////////////////
    // Configure AK09916

    // Test WHO_AM_I
    int tries = 0;
    int max_tries = 5;
    while(tries < max_tries) {
        if(test_who_i_am_mag())
            break;
        if(!is_i2c_bus_)
            i2c_master_reset();
        tries++;
        usleep(10000);
    }

    ret &= set_mag_mode(mag_mode_);

    return ret;
}