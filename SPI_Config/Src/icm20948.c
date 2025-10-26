/*
 * ICM20948 9-Axis IMU Driver for STM32F411 via SPI
 * Gyroscope + Accelerometer + Magnetometer
 * Compatible with custom SPI configuration
 */

#include "stm32f4xx.h"
#include "spi.h"
#include "tim.h"
#include "icm20948.h"
#include "systick.h"
#include <math.h>

// ============= ICM20948 Register Map =============
// User Bank 0
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_LP_CONFIG          0x05
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_INT_PIN_CFG        0x0F
#define ICM20948_INT_ENABLE         0x10
#define ICM20948_INT_ENABLE_1       0x11
#define ICM20948_INT_ENABLE_2       0x12
#define ICM20948_INT_ENABLE_3       0x13
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_ACCEL_XOUT_L       0x2E
#define ICM20948_ACCEL_YOUT_H       0x2F
#define ICM20948_ACCEL_YOUT_L       0x30
#define ICM20948_ACCEL_ZOUT_H       0x31
#define ICM20948_ACCEL_ZOUT_L       0x32
#define ICM20948_GYRO_XOUT_H        0x33
#define ICM20948_GYRO_XOUT_L        0x34
#define ICM20948_GYRO_YOUT_H        0x35
#define ICM20948_GYRO_YOUT_L        0x36
#define ICM20948_GYRO_ZOUT_H        0x37
#define ICM20948_GYRO_ZOUT_L        0x38
#define ICM20948_TEMP_OUT_H         0x39
#define ICM20948_TEMP_OUT_L         0x3A
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_REG_BANK_SEL       0x7F

// User Bank 2
#define ICM20948_GYRO_SMPLRT_DIV    0x00
#define ICM20948_GYRO_CONFIG_1      0x01
#define ICM20948_GYRO_CONFIG_2      0x02
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG       0x14
#define ICM20948_ACCEL_CONFIG_2     0x15

// User Bank 3
#define ICM20948_I2C_MST_CTRL       0x01
#define ICM20948_I2C_SLV0_ADDR      0x03
#define ICM20948_I2C_SLV0_REG       0x04
#define ICM20948_I2C_SLV0_CTRL      0x05
#define ICM20948_I2C_SLV0_DO        0x06

// Magnetometer (AK09916) Registers
#define AK09916_ADDRESS             0x0C
#define AK09916_WIA2                0x01
#define AK09916_ST1                 0x10
#define AK09916_HXL                 0x11
#define AK09916_HXH                 0x12
#define AK09916_HYL                 0x13
#define AK09916_HYH                 0x14
#define AK09916_HZL                 0x15
#define AK09916_HZH                 0x16
#define AK09916_ST2                 0x18
#define AK09916_CNTL2               0x31
#define AK09916_CNTL3               0x32

// Constants
#define ICM20948_DEVICE_ID          0xEA
#define AK09916_DEVICE_ID           0x09

// Gyro và Accel Scale Factors
#define GYRO_SCALE_250DPS           (250.0f / 32768.0f)
#define GYRO_SCALE_500DPS           (500.0f / 32768.0f)
#define GYRO_SCALE_1000DPS          (1000.0f / 32768.0f)
#define GYRO_SCALE_2000DPS          (2000.0f / 32768.0f)

#define ACCEL_SCALE_2G              (2.0f / 32768.0f)
#define ACCEL_SCALE_4G              (4.0f / 32768.0f)
#define ACCEL_SCALE_8G              (8.0f / 32768.0f)
#define ACCEL_SCALE_16G             (16.0f / 32768.0f)

#define MAG_SCALE_FACTOR            (4912.0f / 32768.0f)


// Scale factors (có thể thay đổi theo config)
static float gyro_scale = GYRO_SCALE_250DPS;
static float accel_scale = ACCEL_SCALE_2G;

/**
 * @brief Đọc 1 byte từ register
 * @param reg: địa chỉ register
 * @return giá trị đọc được
 */
uint8_t ICM20948_ReadByte(uint8_t reg) {

    uint8_t tx_data = reg | 0x80; // Set bit 7 để đọc
    uint8_t rx_data;
    
    cs_enable();
    delay_us(1);
    
    spi1_transmit(&tx_data, 1);
    spi1_receive(&rx_data, 1);
    
    delay_us(1);
    cs_disable();
    
    return rx_data;
}

/**
 * @brief Ghi 1 byte vào register
 * @param reg: địa chỉ register
 * @param data: dữ liệu cần ghi
 */
void ICM20948_WriteByte(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F; // Clear bit 7 để ghi
    tx_data[1] = data;
    
    cs_enable();
    delay_us(1);
    
    spi1_transmit(tx_data, 2);
    
    delay_us(1);
    cs_disable();
}

/**
 * @brief Đọc nhiều byte từ register
 * @param reg: địa chỉ register bắt đầu
 * @param data: buffer lưu dữ liệu
 * @param len: số byte cần đọc
 */
void ICM20948_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx_data = reg | 0x80;
    
    cs_enable();
    delay_us(1);
    
    spi1_transmit(&tx_data, 1);
    spi1_receive(data, len);
    
    delay_us(1);
    cs_disable();
}

/**
 * @brief Chọn User Bank (0, 1, 2, 3)
 * @param bank: bank cần chọn
 */
void ICM20948_SelectBank(uint8_t bank) {
    ICM20948_WriteByte(ICM20948_REG_BANK_SEL, bank << 4);
}

// ============= ICM20948 Magnetometer Functions =============

/**
 * @brief Ghi vào Magnetometer qua I2C Master
 * @param reg: địa chỉ register của magnetometer
 * @param data: dữ liệu cần ghi
 */
void ICM20948_MagWrite(uint8_t reg, uint8_t data) {
    ICM20948_SelectBank(3);
    
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_DO, data);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81); // Enable + 1 byte
    
    delay_ms(10);
    ICM20948_SelectBank(0);
}

/**
 * @brief Đọc từ Magnetometer qua I2C Master
 * @param reg: địa chỉ register của magnetometer
 * @return giá trị đọc được
 */
uint8_t ICM20948_MagRead(uint8_t reg) {
    ICM20948_SelectBank(3);
    
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81); // Enable + 1 byte
    
    delay_ms(10);
    ICM20948_SelectBank(0);
    
    return ICM20948_ReadByte(ICM20948_EXT_SLV_SENS_DATA_00);
}

// ============= ICM20948 Initialization =============

/**
 * @brief Khởi tạo ICM20948
 * @return 1 nếu thành công, 0 nếu thất bại
 */
uint8_t ICM20948_Init(void) {
    uint8_t whoami;
    
    // Delay sau power-up
    delay_ms(100);
    
    // Kiểm tra WHO_AM_I
    ICM20948_SelectBank(0);
    whoami = ICM20948_ReadByte(ICM20948_WHO_AM_I);
    
    if(whoami != ICM20948_DEVICE_ID) {
        return 0; // Lỗi: không phát hiện ICM20948
    }
    
    // Reset device
    ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x80);
    delay_ms(100);
    
    // Wake up device và chọn best available clock
    ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x01);
    delay_ms(10);
    
    // Disable I2C interface (chỉ dùng SPI)
    ICM20948_WriteByte(ICM20948_USER_CTRL, 0x10);
    delay_ms(10);
    
    // Enable Accel & Gyro
    ICM20948_WriteByte(ICM20948_PWR_MGMT_2, 0x00);
    delay_ms(10);
    
    // ===== Cấu hình Gyro (Bank 2) =====
    ICM20948_SelectBank(2);
    
    // Gyro sample rate divider
    ICM20948_WriteByte(ICM20948_GYRO_SMPLRT_DIV, 0x04);
    
    // Gyro config: ±250 dps, DLPF enable
    ICM20948_WriteByte(ICM20948_GYRO_CONFIG_1, 0x01);
    
    // Gyro averaging filter
    ICM20948_WriteByte(ICM20948_GYRO_CONFIG_2, 0x00);
    delay_ms(10);
    
    // ===== Cấu hình Accel (Bank 2) =====
    // Accel sample rate divider
    ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
    ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_2, 0x04);
    
    // Accel config: ±2g, DLPF enable
    ICM20948_WriteByte(ICM20948_ACCEL_CONFIG, 0x01);
    
    // Accel averaging filter
    ICM20948_WriteByte(ICM20948_ACCEL_CONFIG_2, 0x00);
    delay_ms(10);
    
    // ===== Bật I2C Master Mode (Bank 0) =====
    ICM20948_SelectBank(0);
    ICM20948_WriteByte(ICM20948_USER_CTRL, 0x20);
    delay_ms(10);
    
    // ===== Cấu hình I2C Master (Bank 3) =====
    ICM20948_SelectBank(3);
    ICM20948_WriteByte(ICM20948_I2C_MST_CTRL, 0x07); // 400kHz
    delay_ms(10);
    
    // ===== Khởi tạo Magnetometer =====
    // Reset Magnetometer
    ICM20948_MagWrite(AK09916_CNTL3, 0x01);
    delay_ms(100);
    
    // Kiểm tra WHO_AM_I của Magnetometer
    uint8_t mag_id = ICM20948_MagRead(AK09916_WIA2);
    if(mag_id != AK09916_DEVICE_ID) {
        // Magnetometer không phát hiện, nhưng tiếp tục init
    }
    
    // Set Magnetometer to Continuous Mode 4 (100Hz)
    ICM20948_MagWrite(AK09916_CNTL2, 0x08);
    delay_ms(10);
    
    // ===== Cấu hình đọc Magnetometer tự động =====
    ICM20948_SelectBank(3);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, AK09916_HXL);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x89); // Enable + 9 bytes
    
    ICM20948_SelectBank(0);
    delay_ms(100);
    
    return 1; // Khởi tạo thành công
}

// ============= ICM20948 Data Reading =============

/**
 * @brief Đọc dữ liệu Accelerometer
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 */
void ICM20948_ReadAccel(ICM20948_Data *data) {
    uint8_t buffer[6];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_ACCEL_XOUT_H, buffer, 6);
    
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Chuyển đổi sang g
    data->accel_x_g = data->accel_x * accel_scale;
    data->accel_y_g = data->accel_y * accel_scale;
    data->accel_z_g = data->accel_z * accel_scale;
}

/**
 * @brief Đọc dữ liệu Gyroscope
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 */
void ICM20948_ReadGyro(ICM20948_Data *data) {
    uint8_t buffer[6];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_GYRO_XOUT_H, buffer, 6);
    
    data->gyro_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->gyro_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->gyro_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Chuyển đổi sang dps (degrees per second)
    data->gyro_x_dps = data->gyro_x * gyro_scale;
    data->gyro_y_dps = data->gyro_y * gyro_scale;
    data->gyro_z_dps = data->gyro_z * gyro_scale;
}

/**
 * @brief Đọc dữ liệu Magnetometer
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 */
void ICM20948_ReadMag(ICM20948_Data *data) {
    uint8_t buffer[9];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_EXT_SLV_SENS_DATA_00, buffer, 9);
    
    // Kiểm tra data ready (ST1 bit 0)
    if(buffer[0] & 0x01) {
        // Magnetometer data format: Little Endian
        data->mag_x = (int16_t)((buffer[2] << 8) | buffer[1]);
        data->mag_y = (int16_t)((buffer[4] << 8) | buffer[3]);
        data->mag_z = (int16_t)((buffer[6] << 8) | buffer[5]);
        
        // Chuyển đổi sang µT (microtesla)
        data->mag_x_ut = data->mag_x * MAG_SCALE_FACTOR;
        data->mag_y_ut = data->mag_y * MAG_SCALE_FACTOR;
        data->mag_z_ut = data->mag_z * MAG_SCALE_FACTOR;
    }
}

/**
 * @brief Đọc nhiệt độ
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 */
void ICM20948_ReadTemp(ICM20948_Data *data) {
    uint8_t buffer[2];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_TEMP_OUT_H, buffer, 2);
    
    data->temperature = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    // Chuyển đổi sang độ C
    // Formula: Temp_degC = ((Temp_out - RoomTemp_Offset) / Temp_Sensitivity) + 21
    data->temp_c = (data->temperature / 333.87f) + 21.0f;
}

/**
 * @brief Đọc tất cả dữ liệu cảm biến
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 */
void ICM20948_ReadAll(ICM20948_Data *data) {
    ICM20948_ReadAccel(data);
    ICM20948_ReadGyro(data);
    ICM20948_ReadMag(data);
    ICM20948_ReadTemp(data);
}

/**
 * @brief Tính góc nghiêng từ dữ liệu accelerometer
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 * @param roll: con trỏ lưu góc roll (độ)
 * @param pitch: con trỏ lưu góc pitch (độ)
 */
void ICM20948_GetAngles(ICM20948_Data *data, float *roll, float *pitch) {
    *roll = atan2f(data->accel_y_g, data->accel_z_g) * 180.0f / 3.14159265f;
    *pitch = atan2f(-data->accel_x_g, sqrtf(data->accel_y_g * data->accel_y_g + 
                                             data->accel_z_g * data->accel_z_g)) * 180.0f / 3.14159265f;
}

/**
 * @brief Tính hướng la bàn từ dữ liệu magnetometer
 * @param data: con trỏ đến cấu trúc ICM20948_Data
 * @return góc hướng (0-360 độ)
 */
float ICM20948_GetHeading(ICM20948_Data *data) {
    float heading = atan2f(data->mag_y_ut, data->mag_x_ut) * 180.0f / 3.14159265f;
    
    if(heading < 0) {
        heading += 360.0f;
    }
    
    return heading;
}

// ============= Main Program Example =============

// int main(void) {
//     ICM20948_Data imu_data;
//     float roll, pitch, heading;
    
//     // Khởi tạo SPI1
//     spi1_init();
//     spi1_config();
    
//     // CS mặc định HIGH
//     cs_disable();
    
//     delay_ms(100);
    
//     // Khởi tạo ICM20948
//     if(!ICM20948_Init()) {
//         // Lỗi khởi tạo - LED nháy hoặc xử lý lỗi
//         while(1) {
//             // Trapped here
//             delay_ms(500);
//         }
//     }
    
//     // Khởi tạo thành công
//     while(1) {
//         // Đọc tất cả dữ liệu
//         ICM20948_ReadAll(&imu_data);
        
//         // Tính góc nghiêng
//         ICM20948_GetAngles(&imu_data, &roll, &pitch);
        
//         // Tính hướng la bàn
//         heading = ICM20948_GetHeading(&imu_data);
        
//         delay_ms(100); // Đọc mỗi 100ms (10Hz)
//     }
// }