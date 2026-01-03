/**
 ******************************************************************************
 * @file    icm20948.h
 * @author  Duong Nguyen (Modified by Gemini)
 * @brief   Header file for ICM-20948 9-Axis MEMS MotionTracking Device.
 * Dựa trên TDK/InvenSense Datasheet DS-000189 v1.6.
 ******************************************************************************
 */

#ifndef ICM20948_H_
#define ICM20948_H_

#include <stdint.h>
#include "spi.h"
#include "systick.h" 
/* -------------------------------------------------------------------------- */
/* I2C ADDRESSES & WHO_AM_I VALUES                                            */
/* -------------------------------------------------------------------------- */
#define ICM20948_I2C_ADDR_AD0_LOW    0x68 /*<! AD0 pin là logic low */
#define ICM20948_I2C_ADDR_AD0_HIGH   0x69 /*<! AD0 pin là logic high */
#define AK09916_I2C_ADDR             0x0C /*<! Địa chỉ I2C của từ kế */

#define ICM20948_WHO_AM_I_VAL        0xEA /*<! Giá trị trả về từ ICM20948 WHO_AM_I */
#define AK09916_WHO_AM_I_VAL         0x09 /*<! Giá trị trả về từ AK09916 WIA2 */

/* -------------------------------------------------------------------------- */
/* REGISTER BANK SELECT                                                       */
/* -------------------------------------------------------------------------- */
#define ICM20948_REG_BANK_SEL        0x7F /*<! Địa chỉ thanh ghi Bank Select */

#define ICM20948_BANK_0              (0x00 << 4)
#define ICM20948_BANK_1              (0x01 << 4)
#define ICM20948_BANK_2              (0x02 << 4)
#define ICM20948_BANK_3              (0x03 << 4)

/* -------------------------------------------------------------------------- */
/* USER BANK 0 REGISTERS                                                      */
/* -------------------------------------------------------------------------- */
#define ICM20948_REG_B0_WHO_AM_I             0x00
#define ICM20948_REG_B0_USER_CTRL            0x03
#define ICM20948_REG_B0_LP_CONFIG            0x05
#define ICM20948_REG_B0_PWR_MGMT_1           0x06
#define ICM20948_REG_B0_PWR_MGMT_2           0x07
#define ICM20948_REG_B0_INT_PIN_CFG          0x0F
#define ICM20948_REG_B0_INT_ENABLE           0x10
#define ICM20948_REG_B0_INT_ENABLE_1         0x11
#define ICM20948_REG_B0_I2C_MST_STATUS       0x17
#define ICM20948_REG_B0_INT_STATUS           0x19
#define ICM20948_REG_B0_INT_STATUS_1         0x1A
#define ICM20948_REG_B0_ACCEL_XOUT_H         0x2D
#define ICM20948_REG_B0_ACCEL_XOUT_L         0x2E
#define ICM20948_REG_B0_ACCEL_YOUT_H         0x2F
#define ICM20948_REG_B0_ACCEL_YOUT_L         0x30
#define ICM20948_REG_B0_ACCEL_ZOUT_H         0x31
#define ICM20948_REG_B0_ACCEL_ZOUT_L         0x32
#define ICM20948_REG_B0_GYRO_XOUT_H          0x33
#define ICM20948_REG_B0_GYRO_XOUT_L          0x34
#define ICM20948_REG_B0_GYRO_YOUT_H          0x35
#define ICM20948_REG_B0_GYRO_YOUT_L          0x36
#define ICM20948_REG_B0_GYRO_ZOUT_H          0x37
#define ICM20948_REG_B0_GYRO_ZOUT_L          0x38
#define ICM20948_REG_B0_TEMP_OUT_H           0x39
#define ICM20948_REG_B0_TEMP_OUT_L           0x3A
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_REG_B0_FIFO_EN_2            0x67
#define ICM20948_REG_B0_FIFO_COUNTH          0x70
#define ICM20948_REG_B0_FIFO_COUNTL          0x71
#define ICM20948_REG_B0_FIFO_R_W             0x72
#define ICM20948_REG_B0_DATA_RDY_STATUS      0x74

/* -------------------------------------------------------------------------- */
/* USER BANK 1 REGISTERS                                                      */
/* -------------------------------------------------------------------------- */
/* (User Bank 1 registers omitted for brevity if not used in core functions) */

/* -------------------------------------------------------------------------- */
/* USER BANK 2 REGISTERS                                                      */
/* -------------------------------------------------------------------------- */
#define ICM20948_REG_B2_GYRO_SMPLRT_DIV      0x00
#define ICM20948_REG_B2_GYRO_CONFIG_1        0x01
#define ICM20948_REG_B2_ACCEL_SMPLRT_DIV_1   0x10
#define ICM20948_REG_B2_ACCEL_SMPLRT_DIV_2   0x11
#define ICM20948_REG_B2_ACCEL_CONFIG         0x14

/* -------------------------------------------------------------------------- */
/* USER BANK 3 REGISTERS                                                      */
/* -------------------------------------------------------------------------- */
#define ICM20948_REG_B3_I2C_MST_CTRL         0x01
#define ICM20948_REG_B3_I2C_SLV0_ADDR        0x03
#define ICM20948_REG_B3_I2C_SLV0_REG         0x04
#define ICM20948_REG_B3_I2C_SLV0_CTRL        0x05
#define ICM20948_REG_B3_I2C_SLV0_DO          0x06
#define ICM20948_REG_B3_I2C_SLV4_ADDR        0x13
#define ICM20948_REG_B3_I2C_SLV4_REG         0x14
#define ICM20948_REG_B3_I2C_SLV4_CTRL        0x15
#define ICM20948_REG_B3_I2C_SLV4_DO          0x16

/* -------------------------------------------------------------------------- */
/* MAGNETOMETER (AK09916) REGISTERS                                           */
/* -------------------------------------------------------------------------- */
#define AK09916_REG_WIA2             0x01 /*<! Device ID */
#define AK09916_REG_ST1              0x10 /*<! Status 1 */
#define AK09916_REG_HXL              0x11 /*<! X-axis data (L) */
#define AK09916_REG_HXH              0x12 /*<! X-axis data (H) */
#define AK09916_REG_HYL              0x13 /*<! Y-axis data (L) */
#define AK09916_REG_HYH              0x14 /*<! Y-axis data (H) */
#define AK09916_REG_HZL              0x15 /*<! Z-axis data (L) */
#define AK09916_REG_HZH              0x16 /*<! Z-axis data (H) */
#define AK09916_REG_ST2              0x18 /*<! Status 2 */
#define AK09916_REG_CNTL2            0x31 /*<! Control 2 (Operation mode) */
#define AK09916_REG_CNTL3            0x32 /*<! Control 3 (Soft reset) */

/* -------------------------------------------------------------------------- */
/* BIT-FIELD DEFINITIONS AND MASKS                                            */
/* -------------------------------------------------------------------------- */

/* MASK: ICM20948_REG_B0_PWR_MGMT_1 */
#define PWR_MGMT_1_DEVICE_RESET_BIT  (1 << 7)
#define PWR_MGMT_1_SLEEP_BIT         (1 << 6)
#define PWR_MGMT_1_CLKSEL_MASK       (0x07)
#define PWR_MGMT_1_CLKSEL_BEST       (0x01)

/* MASK: ICM20948_REG_B0_USER_CTRL */
#define USER_CTRL_I2C_MST_EN_BIT     (1 << 5)

/* MASK: ICM20948_REG_B2_GYRO_CONFIG_1 */
#define GYRO_CONFIG_1_FS_SEL_MASK    (0x06)
#define GYRO_CONFIG_1_FCHOICE_BIT    (1 << 0)

/* MASK: ICM20948_REG_B2_ACCEL_CONFIG */
#define ACCEL_CONFIG_FS_SEL_MASK     (0x06)
#define ACCEL_CONFIG_FCHOICE_BIT     (1 << 0)

/* MASK: ICM20948_REG_B3_I2C_MST_CTRL */
#define I2C_MST_CTRL_CLK_400KHZ      (0x07)

/* MASK: ICM20948_REG_B3_I2C_SLV_ADDR */
#define I2C_SLV_ADDR_READ_BIT        (0x80)

/* MASK: ICM20948_REG_B3_I2C_SLV_CTRL */
#define I2C_SLV_CTRL_EN_BIT          (0x80)

/* MASK: AK09916_REG_ST1 */
#define AK09916_ST1_DRDY_BIT         (1 << 0)

/* MASK: AK09916_REG_ST2 */
#define AK09916_ST2_HOFL_BIT         (1 << 3)

/* MASK: AK09916_REG_CNTL2 */
#define AK09916_CNTL2_MODE_PWR_DOWN  (0x00)
#define AK09916_CNTL2_MODE_SINGLE    (0x01)
#define AK09916_CNTL2_MODE_CONT_100HZ (0x08) /*<! 100 Hz Continuous mode */

/* MASK: AK09916_REG_CNTL3 */
#define AK09916_CNTL3_SRST_BIT       (1 << 0)

/* Cờ R/W cho giao thức SPI của ICM-20948 */
#define SPI_READ_FLAG   0x80 /*<! Bit 7 = 1 để đọc */
#define SPI_WRITE_FLAG  0x7F /*<! Bit 7 = 0 để ghi (AND mask) */

/* *
 * Các hằng số độ nhạy (LSB/unit) dựa trên cấu hình trong icm20948_init()
 */

/* * ACCEL FSR = ±4g --> 8192 LSB/g  */
#define ACCEL_SENSITIVITY_FSR_4G    8192.0f

/* * GYRO FSR = ±2000dps --> 16.4 LSB/dps  */
#define GYRO_SENSITIVITY_FSR_2000   16.4f

/* * MAG SENSITIVITY = 0.15 µT/LSB  */
#define MAG_SENSITIVITY             0.15f

/* * TEMP SENSITIVITY = 333.87 LSB/°C  */
#define TEMP_SENSITIVITY            333.87f
#define TEMP_OFFSET_C               21.0f 

/* -------------------------------------------------------------------------- */
/* PUBLIC DATA TYPES                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Cấu trúc lưu trữ dữ liệu thô (raw 16-bit) từ các cảm biến.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    
    int16_t temp;
} icm20948_raw_data_t;

/**
 * @brief Cấu trúc lưu trữ dữ liệu đã chuyển đổi (scaled) sang đơn vị vật lý.
 */
typedef struct {
    float accel_x_g;  /*<! Đơn vị (g) */
    float accel_y_g;  /*<! Đơn vị (g) */
    float accel_z_g;  /*<! Đơn vị (g) */
    
    float gyro_x_dps; /*<! Đơn vị (degrees/sec) */
    float gyro_y_dps; /*<! Đơn vị (degrees/sec) */
    float gyro_z_dps; /*<! Đơn vị (degrees/sec) */
    
    float mag_x_ut;   /*<! Đơn vị (micro-Tesla) */
    float mag_y_ut;   /*<! Đơn vị (micro-Tesla) */
    float mag_z_ut;   /*<! Đơn vị (micro-Tesla) */
    
    float temp_c;     /*<! Đơn vị (degrees Celsius) */
} icm20948_scaled_data_t;

/**
 * @brief Enum định nghĩa Full-Scale Range cho Gyroscope.
 */
typedef enum 
{
    ICM20948_GYRO_FSR_250_DPS  = 0, /*<! ±250 dps */
    ICM20948_GYRO_FSR_500_DPS  = 1, /*<! ±500 dps */
    ICM20948_GYRO_FSR_1000_DPS = 2, /*<! ±1000 dps */
    ICM20948_GYRO_FSR_2000_DPS = 3  /*<! ±2000 dps */
} icm20948_gyro_fsr_t;

/**
 * @brief Enum định nghĩa Full-Scale Range cho Accelerometer.
 */
typedef enum 
{
    ICM20948_ACCEL_FSR_2G  = 0, /*<! ±2g */
    ICM20948_ACCEL_FSR_4G  = 1, /*<! ±4g */
    ICM20948_ACCEL_FSR_8G  = 2, /*<! ±8g */
    ICM20948_ACCEL_FSR_16G = 3  /*<! ±16g */
} icm20948_accel_fsr_t;

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Khởi tạo cảm biến ICM20948 và từ kế AK09916.
 *
 * @return 0 nếu thành công, -1 nếu không phát hiện được cảm biến.
 */
int8_t icm20948_init(void);

/**
 * @brief Chuyển đổi dữ liệu thô sang dữ liệu vật lý (scaled).
 *
 * @param p_raw_data Con trỏ tới dữ liệu thô đầu vào.
 * @param p_scaled_data Con trỏ tới cấu trúc để lưu dữ liệu đã chuyển đổi.
 */
void icm20948_convert_to_scaled(icm20948_raw_data_t *p_raw_data, icm20948_scaled_data_t *p_scaled_data);

/**
 * @brief Đọc dữ liệu thô từ kế (đã được I2C Master đọc và lưu vào Bank 0).
 */
void icm20948_read_mag_raw(icm20948_raw_data_t *p_data_out);

/**
 * @brief Đọc dữ liệu thô (Accel, Gyro, Temp) từ Bank 0.
 */
void icm20948_read_all_raw(icm20948_raw_data_t *p_data_out);


#endif /* ICM20948_H_ */