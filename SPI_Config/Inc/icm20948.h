/**
 * @file icm20948.h
 * @brief Header file for the ICM20948 9-Axis IMU Driver for STM32.
 * File này chứa các cấu trúc dữ liệu public và các nguyên mẫu hàm (prototypes)
 * cho phép các file khác (như main.c) tương tác với cảm biến.
 */

#ifndef __ICM20948_H
#define __ICM20948_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // Để định nghĩa các kiểu int16_t, uint8_t
#include <math.h>   // Để file .c sử dụng M_PI hoặc tương tự (nếu cần)

// ============= Cấu trúc dữ liệu Public =============

/**
 * @brief Cấu trúc lưu trữ dữ liệu thô (raw) và dữ liệu đã chuyển đổi
 * từ cảm biến ICM20948.
 */
typedef struct {
    // Dữ liệu thô (Raw)
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t temperature;
    
    // Dữ liệu đã chuyển đổi (Converted)
    float accel_x_g;  // Đơn vị Gs
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps; // Đơn vị degrees per second
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_ut;   // Đơn vị microtesla (µT)
    float mag_y_ut;
    float mag_z_ut;
    float temp_c;     // Đơn vị độ C
} ICM20948_Data;


// ============= Nguyên mẫu hàm (Function Prototypes) =============

/**
 * @brief Khởi tạo cảm biến ICM20948.
 * @note  Hàm này thực hiện reset, cấu hình clock, bật cảm biến,
 * cấu hình Gyro, Accel, và I2C Master để đọc Magnetometer.
 * @return 1 nếu khởi tạo thành công, 0 nếu thất bại (sai WHO_AM_I).
 */
uint8_t ICM20948_Init(void);

/**
 * @brief Đọc dữ liệu Accelerometer (cả thô và chuyển đổi).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data để lưu dữ liệu.
 */
void ICM20948_ReadAccel(ICM20948_Data *data);

/**
 * @brief Đọc dữ liệu Gyroscope (cả thô và chuyển đổi).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data để lưu dữ liệu.
 */
void ICM20948_ReadGyro(ICM20948_Data *data);

/**
 * @brief Đọc dữ liệu Magnetometer (cả thô và chuyển đổi).
 * @note  Đọc từ các thanh ghi External Sensor Data (do I2C Master tự động đọc).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data để lưu dữ liệu.
 */
void ICM20948_ReadMag(ICM20948_Data *data);

/**
 * @brief Đọc dữ liệu nhiệt độ (cả thô và chuyển đổi).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data để lưu dữ liệu.
 */
void ICM20948_ReadTemp(ICM20948_Data *data);

/**
 * @brief Đọc tất cả dữ liệu từ cảm biến (Accel, Gyro, Mag, Temp).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data để lưu dữ liệu.
 */
void ICM20948_ReadAll(ICM20948_Data *data);

/**
 * @brief Tính góc nghiêng (roll và pitch) từ dữ liệu accelerometer.
 * @param data: Con trỏ đến cấu trúc ICM20948_Data chứa dữ liệu accel đã đọc.
 * @param roll: Con trỏ (output) để lưu trữ góc roll (tính bằng độ).
 * @param pitch: Con trỏ (output) để lưu trữ góc pitch (tính bằng độ).
 */
void ICM20948_GetAngles(ICM20948_Data *data, float *roll, float *pitch);

/**
 * @brief Tính góc hướng la bàn (heading/yaw) từ dữ liệu magnetometer.
 * @note  Để có kết quả chính xác, cần hiệu chỉnh (calibrate) magnetometer
 * và sử dụng bộ lọc (ví dụ: Tilt Compensation).
 * @param data: Con trỏ đến cấu trúc ICM20948_Data chứa dữ liệu mag đã đọc.
 * @return Góc hướng (0-360 độ).
 */
float ICM20948_GetHeading(ICM20948_Data *data);


#ifdef __cplusplus
}
#endif

#endif /* __ICM20948_H */