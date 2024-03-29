//
// Created by lemezoth on 02/12/23.
//

#ifndef ICM20948_REGISTER_H
#define ICM20948_REGISTER_H

#define ICM20948_WHO_AM_I 0xEA

// IMU registers Bank0
#define ICM20948_BK0_WHO_AM_I 0
#define ICM20948_BK0_USER_CTRL 3
#define ICM20948_BK0_LP_CONFIG 5
#define ICM20948_BK0_PWR_MGMT_1 6
#define ICM20948_BK0_PWR_MGMT_2 7
#define ICM20948_BK0_INT_PIN_CFG  15
#define ICM20948_BK0_INT_ENABLE  16
#define ICM20948_BK0_INT_ENABLE_1  17
#define ICM20948_BK0_INT_ENABLE_2  18
#define ICM20948_BK0_INT_ENABLE_3  19
#define ICM20948_BK0_I2C_MST_STATUS  23
#define ICM20948_BK0_INT_STATUS  25
#define ICM20948_BK0_INT_STATUS_1  26
#define ICM20948_BK0_INT_STATUS_2  27
#define ICM20948_BK0_INT_STATUS_3  28
#define ICM20948_BK0_DELAY_TIMEH  40
#define ICM20948_BK0_DELAY_TIMEL  41
#define ICM20948_BK0_ACCEL_XOUT_H  45
#define ICM20948_BK0_ACCEL_XOUT_L  46
#define ICM20948_BK0_ACCEL_YOUT_H  47
#define ICM20948_BK0_ACCEL_YOUT_L  48
#define ICM20948_BK0_ACCEL_ZOUT_H  49
#define ICM20948_BK0_ACCEL_ZOUT_L  50
#define ICM20948_BK0_GYRO_XOUT_H  51
#define ICM20948_BK0_GYRO_XOUT_L  52
#define ICM20948_BK0_GYRO_YOUT_H  53
#define ICM20948_BK0_GYRO_YOUT_L  54
#define ICM20948_BK0_GYRO_ZOUT_H  55
#define ICM20948_BK0_GYRO_ZOUT_L  56
#define ICM20948_BK0_TEMP_OUT_H  57
#define ICM20948_BK0_TEMP_OUT_L  58
#define ICM20948_BK0_EXT_SLV_SENS_DATA_00  59
#define ICM20948_BK0_EXT_SLV_SENS_DATA_23  82
#define ICM20948_BK0_FIFO_EN_1  102
#define ICM20948_BK0_FIFO_EN_2  103
#define ICM20948_BK0_FIFO_RST  104
#define ICM20948_BK0_FIFO_MODE  105
#define ICM20948_BK0_FIFO_COUNTH  112
#define ICM20948_BK0_FIFO_COUNTL  113
#define ICM20948_BK0_FIFO_R_W  114
#define ICM20948_BK0_DATA_RDY_STATUS  116
#define ICM20948_BK0_FIFO_CFG  118
#define ICM20948_BK0_REG_BANK_SEL  127

// IMU registers Bank1
#define ICM20948_BK1_SELF_TEST_X_GYRO  2
#define ICM20948_BK1_SELF_TEST_Y_GYRO  3
#define ICM20948_BK1_SELF_TEST_Z_GYRO  4
#define ICM20948_BK1_SELF_TEST_X_ACCEL  14
#define ICM20948_BK1_SELF_TEST_Y_ACCEL  15
#define ICM20948_BK1_SELF_TEST_Z_ACCEL  16
#define ICM20948_BK1_XA_OFFS_H  19
#define ICM20948_BK1_XA_OFFS_L  20
#define ICM20948_BK1_YA_OFFS_H  21
#define ICM20948_BK1_YA_OFFS_L  22
#define ICM20948_BK1_ZA_OFFS_H  23
#define ICM20948_BK1_ZA_OFFS_L  24
#define ICM20948_BK1_TIMEBASE_CORRECTION_PLL  40
#define ICM20948_BK1_REG_BANK_SEL  127

// IMU registers Bank2
#define ICM20948_BK2_GYRO_SMPLRT_DIV 0
#define ICM20948_BK2_GYRO_CONFIG_1  1
#define ICM20948_BK2_GYRO_CONFIG_2  2
#define ICM20948_BK2_XG_OFFS_USRH  3
#define ICM20948_BK2_XG_OFFS_USRL  4
#define ICM20948_BK2_YG_OFFS_USRH  5
#define ICM20948_BK2_YG_OFFS_USRL  6
#define ICM20948_BK2_ZG_OFFS_USRH  7
#define ICM20948_BK2_ZG_OFFS_USRL  8
#define ICM20948_BK2_ODR_ALIGN_EN  9
#define ICM20948_BK2_ACCEL_SMPLRT_DIV_1  16
#define ICM20948_BK2_ACCEL_SMPLRT_DIV_2  17
#define ICM20948_BK2_ACCEL_INTEL_CTRL  18
#define ICM20948_BK2_ACCEL_WOM_THR  19
#define ICM20948_BK2_ACCEL_CONFIG  20
#define ICM20948_BK2_ACCEL_CONFIG_2  21
#define ICM20948_BK2_FSYNC_CONFIG  82
#define ICM20948_BK2_TEMP_CONFIG  83
#define ICM20948_BK2_MOD_CTRL_USR  84

// IMU registers Bank3
#define ICM20948_BK3_I2C_MST_ODR_CONFIG  0
#define ICM20948_BK3_I2C_MST_CTRL  1
#define ICM20948_BK3_I2C_MST_DELAY_CTRL  2
#define ICM20948_BK3_I2C_SLV0_ADDR  3
#define ICM20948_BK3_I2C_SLV0_REG  4
#define ICM20948_BK3_I2C_SLV0_CTRL  5
#define ICM20948_BK3_I2C_SLV0_DO  6
#define ICM20948_BK3_I2C_SLV1_ADDR  7
#define ICM20948_BK3_I2C_SLV1_REG  8
#define ICM20948_BK3_I2C_SLV1_CTRL  9
#define ICM20948_BK3_I2C_SLV1_DO  10
#define ICM20948_BK3_I2C_SLV2_ADDR  11
#define ICM20948_BK3_I2C_SLV2_REG  12
#define ICM20948_BK3_I2C_SLV2_CTRL  13
#define ICM20948_BK3_I2C_SLV2_DO  14
#define ICM20948_BK3_I2C_SLV3_ADDR  15
#define ICM20948_BK3_I2C_SLV3_REG  16
#define ICM20948_BK3_I2C_SLV3_CTRL  17
#define ICM20948_BK3_I2C_SLV3_DO  18
#define ICM20948_BK3_I2C_SLV4_ADDR  19
#define ICM20948_BK3_I2C_SLV4_REG  20
#define ICM20948_BK3_I2C_SLV4_CTRL  21
#define ICM20948_BK3_I2C_SLV4_DO  22
#define ICM20948_BK3_I2C_SLV4_DI  23
#define ICM20948_BK3_REG_BANK_SEL  127

#define GYRO_DLPFCFG_BW196HZ  0    // 3dB BW: 196.6 Hz  NBW: 229.8
#define GYRO_DLPFCFG_BW151HZ  1    // 3dB BW: 151.8 Hz  NBW: 187.6
#define GYRO_DLPFCFG_BW119HZ  2    // 3dB BW: 119.5 Hz  NBW: 154.3
#define GYRO_DLPFCFG_BW51HZ  3     // 3dB BW: 51.2 Hz   NBW: 73.3
#define GYRO_DLPFCFG_BW23HZ  4     // 3dB BW: 23.9 Hz   NBW: 35.9
#define GYRO_DLPFCFG_BW11HZ  5     // 3dB BW: 11.6 Hz   NBW: 17.8
#define GYRO_DLPFCFG_BW5HZ  6      // 3dB BW: 5.7 Hz    NBW: 8.9
#define GYRO_DLPFCFG_BW361HZ  7    // 3dB BW: 361.4 Hz  NBW: 375.6

#define GYRO_FS_SEL_250DPS  0  // +/- 250 °/s full scale
#define GYRO_FS_SEL_500DPS  1
#define GYRO_FS_SEL_1000DPS  2
#define GYRO_FS_SEL_2000DPS  3

#define GYRO_AVGCFG_1X  0
#define GYRO_AVGCFG_2X  1
#define GYRO_AVGCFG_4X  2
#define GYRO_AVGCFG_8X  3
#define GYRO_AVGCFG_16X  4
#define GYRO_AVGCFG_32X  5
#define GYRO_AVGCFG_64X  6
#define GYRO_AVGCFG_128X  7

#define ACCEL_FS_SEL_2G  0
#define ACCEL_FS_SEL_4G  1
#define ACCEL_FS_SEL_8G  2
#define ACCEL_FS_SEL_16G  3

#define ACCEL_DLPFCFG_BW246HZ  0    // 3dB BW: 246.0 Hz  NBW: 265.0
#define ACCEL_DLPFCFG_BW246_HZ  1   // 3dB BW: 246.0 Hz  NBW: 265.0
#define ACCEL_DLPFCFG_BW111HZ  2    // 3dB BW: 111.4 Hz  NBW: 136.0
#define ACCEL_DLPFCFG_BW50HZ  3     // 3dB BW: 50.4 Hz   NBW: 68.8
#define ACCEL_DLPFCFG_BW23HZ  4     // 3dB BW: 23.9 Hz   NBW: 34.4
#define ACCEL_DLPFCFG_BW11HZ  5     // 3dB BW: 11.5 Hz   NBW: 17.0
#define ACCEL_DLPFCFG_BW5HZ  6      // 3dB BW: 5.7 Hz    NBW: 8.3
#define ACCEL_DLPFCFG_BW473HZ  7    // 3dB BW: 473 Hz  NBW: 399

#define ACCEL_DEC3CFG_1X  0
#define ACCEL_DEC3CFG_4X  0
#define ACCEL_DEC3CFG_8X  1
#define ACCEL_DEC3CFG_16X 2
#define ACCEL_DEC3CFG_32X 3

//Magnetometer Registers
#define AK09916_ADDRESS  0x0C
#define AK09916_WHO_AM_I_REG 0x01 // (AKA WIA2) should return 0x09
#define AK09916_WHO_AM_I 0x09 // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     0x30  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    0x31  // Normal (0), Reset (1)
#define AK09916_CNTL3    0x32
#define AK09916_SOFT_RST 0x01


typedef enum
{
    AK09916_mode_power_down = 0b0,
    AK09916_mode_single = 0b1,
    AK09916_mode_cont_10hz = 0b10,  // MODE 1
    AK09916_mode_cont_20hz = 0b100, // MODE 2
    AK09916_mode_cont_50hz = 0b110, // MODE 3 Warning : not logical (see datasheet)
    AK09916_mode_cont_100hz = 0b1000, // MODE 4
    AK09916_mode_self_test = 0b10000,
} AK09916_mode_e;

#endif //ICM20948_REGISTER_H
