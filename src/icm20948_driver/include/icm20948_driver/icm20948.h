//
// Created by lemezoth
//

#ifndef BUILD_ICM20948_H
#define BUILD_ICM20948_H

#include <string>
#include <array>
#include "icm20948_register.h"

using namespace std;

class ICM20948 {
public:

    /**
     * Constructor
     */
    ICM20948() = default;

    /**
     * Destructor
     */
    ~ICM20948();

    /**
     *  Init IMU
     */
    int init();

    /**
     *  Get measure
     */
    int get_measure();

    double acc_x_{}, acc_y_{}, acc_z_{};
    double gyro_x_{}, gyro_y_{}, gyro_z_{};
    double mag_x_{}, mag_y_{}, mag_z_{};

    float temp_{};

    double accel_x_bias_{}, accel_y_bias_{}, accel_z_bias_{};

    double mag_x_old_{}, mag_y_old_{}, mag_z_old_{};
    int nb_not_update = 0;

    bool mag_data_has_been_skipped_ = false;
    bool mag_data_is_ready_ = false;
    bool mag_sensor_magnetic_overflow_ = false;

    uint8_t spi_bus_ = 0;
    uint8_t spi_device_ = 0;

    uint8_t gyro_dlpfcfg_ = GYRO_DLPFCFG_BW23HZ;
    uint8_t gyro_fs_sel_ = GYRO_FS_SEL_250DPS;
    uint8_t gyro_fchoice_ = 1;
    uint8_t gyro_avgcfg_ = GYRO_AVGCFG_128X; // For low power mode only
    uint8_t gyro_smplrt_div_ = 21;           // => 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) | 21 => 50Hz (ODR) | MAX 255

    uint8_t accel_smplrt_div_ = 21; // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) => 1.125 kHz/(1+21) = 52.5Hz (ODR)
    uint8_t accel_dlpfcfg_ = ACCEL_DLPFCFG_BW50HZ;
    uint8_t accel_fs_sel_ = ACCEL_FS_SEL_4G;
    uint8_t accel_fchoice_ = 1;
    uint8_t acc_dec3Cfg_ = ACCEL_DEC3CFG_32X; // For low power mode only

    // Interruption Wakeup On Move (WOM)
    uint8_t accel_intel_en_ = 0;   // Disable wake on motion
    uint8_t accel_intel_mode_int_ = 0;
    uint8_t accel_intel_thr_ = 0;

    const std::array<double, 4> accel_sensitivity_ = {2./32767., 4./32767., 8./32767., 16./32767.};
    const std::array<double, 4> gyro_sensitivity_ = {250./32767., 500./32767., 1000./32767., 2000./32767.};
    const std::array<double, 4> gyro_full_scale_ = {250., 500., 1000., 2000.};
    const std::array<double, 4> accel_full_scale_ = {2., 4., 8., 16.};

    const double mag_sensitivity_ = 4912.0/32752.0;
    AK09916_mode_e mag_mode_ = AK09916_mode_cont_50hz;
    uint8_t i2c_smplrt_div_ = 4; // 1.1 kHz/(2^((odr_config[3:0])), 4=>68.75 // 1 ? || MAX = 15

    uint8_t slv4_dly_ = 0x00; // 0x04
    uint8_t i2c_slv0_delay_en_ = 0x00; // 0x01

    uint8_t i2c_frequency_clk_ = 0x07; // 7 recommended
    uint8_t i2c_lp_mode_ = 0x00; // 0x01
    uint8_t i2c_if_dis_ = 0x01;

private:
    int spi_fd_ = 0;
    uint32_t speed_ = 6000000;
    int current_bank_ = -1;

    uint8_t cs_change_ = 0;
    uint16_t delay_usecs_ = 0;


private:

    /**
     *  Open SPI
     */
    int open_spi(const unsigned int &spi_bus, const unsigned int &spi_device);

    /**
     * Read register
     * @param reg
     * @param len
     * @return
     */
    std::vector<uint8_t> readRegister(const uint8_t &reg, const uint32_t &len=1);

    /**
     * Write register
     * @param reg
     * @param data
     * @return
     */
    int writeRegister(const uint8_t &reg, const std::vector<uint8_t> &data);

    /**
     * Write register
     * @param reg
     * @param data
     * @return
     */
    int writeRegister(const uint8_t &reg, const uint8_t &data, bool check_write=true);

    /**
     * imuJoinUserCtrl
     * @param dmpEn
        1: Enables DMP (Digital Motion Processor) features \n
        0: disables
     * @param fifoEn
        1: Enable FIFO operation mode \n
        0: Disable FIFO access from serial interface
     * @param i2cMstEn
        1: Enable i2c master \n
        0: disable i2c master
     * @param i2cIfDis
        1: Reset I2C Slave module and put the serial interface in SPI mode on
     * @param dmpRst
        1: reset DMP module (autoclear bit)
     * @param sramRst
        1: reset SRAM module (autoclear bit)
     * @param i2cMstRst
        1: reset i2c Master (autoclear bit)
     */
    static uint8_t
    imuJoinUserCtrl(uint8_t dmpEn, uint8_t fifoEn, uint8_t i2cMstEn, uint8_t i2cIfDis, uint8_t dmpRst, uint8_t sramRst,
                    uint8_t i2cMstRst);

    /**
     * imuJoinLpConfig
     * @param i2cMstCycle
        1: i2c master will operate in duty cycle mode (ODR -> I2C_MST_ODR_CONFIG) \n
        0: disable i2c master duty cycle
    * @param accelCycle
        1: operate accel in duty cycle mode (ODR -> ACCEL_SMPRT_DIV) \n
        0: disable
    * @param gyroCycle
        1: operate gyro in duty cycle mode (ODR -> ACCEL_SMPRT_DIV) \n
        0: disable
     */
    static uint8_t imuJoinLpConfig(uint8_t i2cMstCycle, uint8_t accelCycle, uint8_t gyroCycle);

    /**
     * imuJoinPwrMgmt1
     * @param deviceRst 1: Reset internal registers - the bit will auto clear
     * @param sleep
          1: chip enters sleep mode, \n
          0: awake chip
     * @param lpEn
         1: turn on low power feature, \n
         0 turn off
     * @param tempDis 1: disable temperature sensor
     * @param clkSel
        0: internal 20MHz oscillator, \n
        1-5: auto selects best available clk source, \n
        6: internal 20 MHz oscillator, \n
        7: Stops the clock and keeps timing generator in reset
     * @return
     */
    static uint8_t imuJoinPwrMgmt1(uint8_t deviceRst, uint8_t sleep, uint8_t lpEn, uint8_t tempDis, uint8_t clkSel);

    /**
     * imuJoinPwrMgmt2
     * @param disableAccel
            1: all accel disable \n
            0: all accel enable
     * @param disableGyro
            1: all gyro disable \n
            0: all gyro enable
     */
    static uint8_t imuJoinPwrMgmt2(uint8_t disableAccel, uint8_t disableGyro);

    /**
     * imuJoinGyroSmplrtDiv
     */
    static uint8_t joinGyroSmplrtDiv(uint8_t gyroSmplrtDiv);

    /**
     * imuJoinGyroConfig1
     * @param gyroDlpfcfg
        GYRO_DLPFCFG_BW196HZ = 0 (3dB BW: 196.6 Hz  NBW: 229.8) \n
        GYRO_DLPFCFG_BW151HZ = 1 (3dB BW: 151.8 Hz  NBW: 187.6) \n
        GYRO_DLPFCFG_BW119HZ = 2 (3dB BW: 119.5 Hz  NBW: 154.3) \n
        GYRO_DLPFCFG_BW51HZ = 3  (3dB BW: 51.2 Hz   NBW: 73.3) \n
        GYRO_DLPFCFG_BW23HZ = 4  (3dB BW: 23.9 Hz   NBW: 35.9) \n
        GYRO_DLPFCFG_BW11HZ = 5  (3dB BW: 11.6 Hz   NBW: 17.8) \n
        GYRO_DLPFCFG_BW5HZ = 6   (3dB BW: 5.7 Hz    NBW: 8.9) \n
        GYRO_DLPFCFG_BW361HZ = 7 (3dB BW: 361.4 Hz  NBW: 375.6)
     * @param gyroFsSel
        GYRO_FS_SEL_250DPS = 0 (+/- 250 °/s full scale) \n
        GYRO_FS_SEL_500DPS = 1 \n
        GYRO_FS_SEL_1000DPS = 2 \n
        GYRO_FS_SEL_2000DPS = 3 \n
     * @param gyroFchoice
        0: Bypass gyro Digital Low Pass Filter \n
            3dB bandwith is 12106 Hz ant Output Data Rate (ODR) is 9000Hz \n
        1: Enable gyro DLPF \n
            3db bandwith is given by gyroDlpfcfg (self.GYRO_DLPFCFG_BWXXX values)
    */
    static uint8_t imuJoinGyroConfig1(uint8_t gyroDlpfCfg, uint8_t gyroFsSel, uint8_t gyroFchoice);

    /**
     * imuJoinGyroConfig2
     * @param xGyroCten X Gyro self-test enable
     * @param yGyroCten
     * @param zGyroCten
     * @param gyroAvgcfg
        Averaging filter configuration settings for low power mode \n
        GYRO_AVGCFG_1X = 0 \n
        GYRO_AVGCFG_2X = 1 \n
        GYRO_AVGCFG_4X = 2 \n
        GYRO_AVGCFG_8X = 3 \n
        GYRO_AVGCFG_16X = 4 \n
        GYRO_AVGCFG_32X = 5 \n
        GYRO_AVGCFG_64X = 6 \n
        GYRO_AVGCFG_128X = 7
     */
    static uint8_t imuJoinGyroConfig2(uint8_t xGyroCten, uint8_t yGyroCten, uint8_t zGyroCten, uint8_t gyroAvgcfg);

    /**
    * imuJoinAccelSmplrtDiv1
    * @param accelSmplrtDiv
     */
    static uint8_t imuJoinAccelSmplrtDiv1(uint16_t accelSmplrtDiv);

    /**
     * imuJoinAccelSmplrtDiv2
     * @param accelSmplrtDiv
     */
    static uint8_t imuJoinAccelSmplrtDiv2(uint16_t accelSmplrtDiv);

    /**
     * imuJoinAccelIntelCtrl
     * @param accelIntEn
        Enable the Wake On Motion logic
     * @param accelIntelModeInt
     * selects WOM algorithm \n
        1: compare the current sample with the previous sample \n
        0: initial sample is stored, all future samples are compared to the initial sample
     */
    static uint8_t imuJoinAccelIntelCtrl(uint8_t accelIntelEn, uint8_t accelIntelModeInt);

    /**
     * imuJoinAccelConfig
     * @param accelDlpfcfg
        Accelerometer low pas filter configuration
     * @param accelFsSel
        Accelerometer full scale select (self.ACCEL_FS_SEL_xG)
     * @param accelFchoice
        0: bypass accel Digital Low Pass Filter \n
        1: Enable accel Digital Low Pass Filter
    */
    static uint8_t imuJoinAccelConfig(uint8_t accelDlpfcfg, uint8_t accelFsSel, uint8_t accelFchoice);

    /**
     * imuJoinAccelConfig2
     * @param axStEnReg X accel self test enable
     * @param ayStEnReg Y accel self test enable
     * @param azStEnReg Z accel self test enable
     * @param dec3Cfg
        Controls the number of samples averaged in the accelerometer decimator \n
        0: Average 1 or 4 samples depending on ACCEL_FCHOICE (see Table 19) \n
        1: Average 8 samples \n
        2: Average 16 samples \n
        3: Average 32 samples \n
     */
    static uint8_t imuJoinAccelConfig2(uint8_t axStEnReg, uint8_t ayStEnReg, uint8_t azStEnReg, uint8_t dec3Cfg);

    /**
     * imuJoinBankId
     * @param bank_id
     * @return
     */
    static uint8_t imuJoinBankId(uint8_t bank_id);

    /**
     *
     * @param multMstEn Enables multi-master capability. When disabled, clocking to the I2C_MST_IF can be
disabled when not in use and the logic to detect lost arbitration is disabled
     * @param i2cMstPNsr This bit controls the I2C Master’s transition from one slave read to the next slave
read. \
0 - There is a restart between reads. \n
1 - There is a stop between reads.
     * @param i2cMstClk Sets I2C master clock frequency (it is recommended to set I2C_MST_CLK = 7)
     * @return
     */
    static uint8_t imuJoinMstCtrl(uint8_t multMstEn, uint8_t i2cMstPNsr, uint8_t i2cMstClk);

    /**
     * imuJoinIntEnable
     * @param regWofEn 1 – Enable wake on FSYNC interrupt
     * @param womIntEn 1 – Enable interrupt for wake on motion to propagate to interrupt pin 1. \n
     *                0 – Function is disabled.
     * @param pllRdyEn 1 – Enable PLL RDY interrupt (PLL RDY means PLL is running and in use as the clock \n
     *               source for the system) to propagate to interrupt pin 1. \n
     *               0 – Function is disabled.
     * @param dmpInt1En 1 – Enable DMP interrupt to propagate to interrupt pin 1. \n
     *               0 – Function is disabled.
     * @param i2cMstIntEn 1 – Enable I2C master interrupt to propagate to interrupt pin 1. \n
     *                 0 – Function is disabled.
     * @return
     */
    static uint8_t imuJoinIntEnable(uint8_t regWofEn, uint8_t womIntEn, uint8_t pllRdyEn, uint8_t dmpInt1En, uint8_t i2cMstIntEn);

    /**
     * imuJoinI2cMstDelayCtrl
     * @param delayEsShadow Delays shadowing of external sensor data until all data is received.
     * @param i2cSlv4DelayEn When enabled, slave 4 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
     * determined by I2C_MST_ODR_CONFIG.
     * @param i2cSlv3DelayEn When enabled, slave 3 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
     * determined by I2C_MST_ODR_CONFIG.
     * @param i2cSlv2DelayEn When enabled, slave 2 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
     * determined by I2C_MST_ODR_CONFIG.
     * @param i2cSlv1DelayEn When enabled, slave 1 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
     * determined by I2C_MST_ODR_CONFIG.
     * @param i2cSlv0DelayEn When enabled, slave 0 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
     * determined by I2C_MST_ODR_CONFIG
     * @return
     */
    static uint8_t imuJoinI2cMstDelayCtrl(uint8_t delayEsShadow, uint8_t i2cSlv4DelayEn, uint8_t i2cSlv3DelayEn,
                                          uint8_t i2cSlv2DelayEn, uint8_t i2cSlv1DelayEn, uint8_t i2cSlv0DelayEn);

    /**
     * imuJoinI2cSlv4Ctrl
     * @param i2cSlv4En 1 – Enable data transfer with this slave at the sample rate. If read command, store
     * data in I2C_SLV4_DI register, if write command, write data stored in I2C_SLV4_DO
     * register. Bit is cleared when a single transfer is complete. Be sure to write
     * I2C_SLV4_DO first.\n
     * 0 – Function is disabled for this slave.
     * @param i2cSlv4IntEn 1 – Enables the completion of the I2C slave 4 data transfer to cause an interrupt. \n
     * 0 – Completion of the I2C slave 4 data transfer will not cause an interrupt.
     * @param i2cSlv4RegDis When set, the transaction does not write a register value, it will only read data, or
     * write data.
     * @param i2cSlv4Dly When enabled via the I2C_MST_DELAY_CTRL, those slaves will only be enabled
     * every 1/(1+I2C_SLV4_DLY) samples as determined by I2C_MST_ODR_CONFIG
     * @return
     */
    static uint8_t imuJoinI2cSlv4Ctrl(uint8_t i2cSlv4En, uint8_t i2cSlv4IntEn, uint8_t i2cSlv4RegDis, uint8_t i2cSlv4Dly);

    /**
     * imuJoinIntPinCfg
     * @param int1Actl 1 - The logic level for INT1 pin is active low.\n
     *                0 - The logic level for INT1 pin is active high.
     * @param int1Open 1 - INT1 pin is configured as open drain.\n
     *               0 - INT1 pin is configured as push-pull.
     * @param int1LatchEn 1 - INT1 pin level held until interrupt status is cleared.\n
     *                 0 - INT1 pin indicates interrupt pulse is width 50 μs.
     * @param int1Anyrd2Clear 1 - Interrupt status in INT_STATUS is cleared (set to 0) if any read operation is
     * performed.\n
     *                     0 - Interrupt status in INT_STATUS is cleared (set to 0) only by reading INT_STATUS register.\n
     *                     This bit only affects the interrupt status bits that are contained in the register
     *                     INT_STATUS, and the corresponding hardware interrupt.
     *                     This bit does not affect the interrupt status bits that are contained in registers
     *                     INT_STATUS_1, INT_STATUS_2, INT_STATUS_3, and the corresponding hardware interrupt.
     * @param int1FsyncLvl 1 - The logic level for the FSYNC pin as an interrupt to the ICM-20948 is active low.\n
     *                   0 - The logic level for the FSYNC pin as an interrupt to the ICM-20948 is active high.
     * @param int1FsyncEn 1 - This enables the FSYNC pin to be used as an interrupt. A transition to the active
     * level described by the ACTL_FSYNC bit will cause an interrupt. The status of the
     * interrupt is read in the I2C Master Status register PASS_THROUGH bit.\n
     *                0 - This disables the FSYNC pin from causing an interrupt.
     * @param int1BypassEn When asserted, the I2C_MASTER interface pins (ES_CL and ES_DA) will go into
     * ‘bypass mode’ when the I2C master interface is disabled
     * @return
     */
    static uint8_t imuJoinIntPinCfg(uint8_t int1Actl, uint8_t int1Open, uint8_t int1LatchEn, uint8_t int1Anyrd2Clear,
                                    uint8_t int1FsyncLvl, uint8_t int1FsyncEn, uint8_t int1BypassEn);

    /**
     * Dump bank
     * @param bank_id
     */
    void dump_bank(const int &bank_id);

    /**
     * dump ak09916
     */
    void dump_ak09916();

    /**
     * dump ICMS20948
     */
     void dump_icm20948();

     /**
      * reset i2c slv4
      */
     int reset_i2c_slv4();

     /**
      * reset i2c
      */
      int reset_i2c();

      /**
       * select bank
       */
      int select_bank(const uint8_t &bank_id);

      /**
       * i2c master reset
       * @return
       */
      int i2c_master_reset();

      /**
       * test who I am magnetometer
       * @return
       */
      bool test_who_i_am_mag();

      /**
       * Set magnetometer mode
       * @param mode
       * @return
       */
        int set_mag_mode(const AK09916_mode_e &mode);

        /**
         * Init magnetometer
         */
        int init_magnetometer();

private:

    int write_i2c_AK09916_byte(const uint8_t &reg, const uint8_t &data);

    int read_i2c_AK09916_byte(const uint8_t &reg, uint8_t &data);


};

#endif //BUILD_ICM20948_H
