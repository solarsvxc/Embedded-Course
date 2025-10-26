/*
 * ICM20948 9-Axis IMU Driver cho STM32F411
 * Không dùng thư viện HAL - Chỉ dùng thanh ghi
 * SPI1: PA5(SCK), PA6(MISO), PA7(MOSI), PA4(CS)
 */

#include "icm20948.h"
#include <math.h>

// ============= ICM20948 Register Map =============
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_USER_CTRL          0x03
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_INT_PIN_CFG        0x0F
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_GYRO_XOUT_H        0x33
#define ICM20948_TEMP_OUT_H         0x39
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

// Magnetometer AK09916
#define AK09916_ADDRESS             0x0C
#define AK09916_WIA2                0x01
#define AK09916_ST1                 0x10
#define AK09916_HXL                 0x11
#define AK09916_CNTL2               0x31
#define AK09916_CNTL3               0x32

#define ICM20948_DEVICE_ID          0xEA
#define AK09916_DEVICE_ID           0x09

// Scale factors
#define GYRO_SCALE_250DPS           (250.0f / 32768.0f)
#define ACCEL_SCALE_2G              (2.0f / 32768.0f)
#define MAG_SCALE_FACTOR            (4912.0f / 32768.0f)

// Cấu trúc dữ liệu
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t temperature;
    
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float mag_x_ut, mag_y_ut, mag_z_ut;
    float temp_c;
} ICM20948_Data;

// ============= GPIO Functions =============

void GPIO_Init(void) {
    // Bật clock cho GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // PA5, PA7: Alternate function (SPI1_SCK, SPI1_MOSI)
    // PA6: Alternate function (SPI1_MISO)
    // PA4: Output (CS)
    
    // PA4: Output push-pull
    GPIOA->MODER &= ~(3U << (4*2));
    GPIOA->MODER |= (1U << (4*2));
    GPIOA->OTYPER &= ~(1U << 4);
    GPIOA->OSPEEDR |= (3U << (4*2)); // High speed
    GPIOA->PUPDR &= ~(3U << (4*2));  // No pull
    
    // PA5, PA6, PA7: Alternate function AF5
    GPIOA->MODER &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER |= (2U << (5*2)) | (2U << (6*2)) | (2U << (7*2));
    GPIOA->AFR[0] &= ~((0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    GPIOA->AFR[0] |= (5U << (5*4)) | (5U << (6*4)) | (5U << (7*4));
    GPIOA->OSPEEDR |= (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2));
}

void CS_Low(void) {
    GPIOA->BSRR = (1U << (4 + 16)); // Reset bit 4
}

void CS_High(void) {
    GPIOA->BSRR = (1U << 4); // Set bit 4
}

// ============= SPI Functions =============

void SPI1_Init(void) {
    // Bật clock cho SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    // Cấu hình SPI1
    // CR1: Master mode, CPOL=1, CPHA=1, BR=fPCLK/16, MSB first
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR;      // Master mode
    SPI1->CR1 |= SPI_CR1_CPOL;      // CPOL = 1
    SPI1->CR1 |= SPI_CR1_CPHA;      // CPHA = 1
    SPI1->CR1 |= (3U << 3);         // BR = fPCLK/16 (~5MHz @ 84MHz PCLK)
    SPI1->CR1 |= SPI_CR1_SSM;       // Software slave management
    SPI1->CR1 |= SPI_CR1_SSI;       // Internal slave select
    
    // CR2: 8-bit data frame
    SPI1->CR2 = 0;
    SPI1->CR2 |= (7U << 8);         // DS = 8 bits
    SPI1->CR2 |= SPI_CR2_FRXTH;     // RXNE event at 8-bit
    
    // Bật SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t SPI1_Transfer(uint8_t data) {
    // Đợi TXE (transmit buffer empty)
    while(!(SPI1->SR & SPI_SR_TXE));
    
    // Gửi dữ liệu
    *(volatile uint8_t *)&SPI1->DR = data;
    
    // Đợi RXNE (receive buffer not empty)
    while(!(SPI1->SR & SPI_SR_RXNE));
    
    // Đọc dữ liệu nhận được
    return *(volatile uint8_t *)&SPI1->DR;
}

// ============= Delay Functions =============

void delay_us(uint32_t us) {
    // Giả sử 84MHz system clock
    for(volatile uint32_t i = 0; i < us * 21; i++);
}

void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// ============= ICM20948 Low-Level Functions =============

uint8_t ICM20948_ReadByte(uint8_t reg) {
    uint8_t rx_data;
    
    CS_Low();
    delay_us(1);
    
    SPI1_Transfer(reg | 0x80); // Bit 7 = 1 để đọc
    rx_data = SPI1_Transfer(0xFF);
    
    delay_us(1);
    CS_High();
    
    return rx_data;
}

void ICM20948_WriteByte(uint8_t reg, uint8_t data) {
    CS_Low();
    delay_us(1);
    
    SPI1_Transfer(reg & 0x7F); // Bit 7 = 0 để ghi
    SPI1_Transfer(data);
    
    delay_us(1);
    CS_High();
}

void ICM20948_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len) {
    CS_Low();
    delay_us(1);
    
    SPI1_Transfer(reg | 0x80);
    for(uint8_t i = 0; i < len; i++) {
        data[i] = SPI1_Transfer(0xFF);
    }
    
    delay_us(1);
    CS_High();
}

void ICM20948_SelectBank(uint8_t bank) {
    ICM20948_WriteByte(ICM20948_REG_BANK_SEL, bank << 4);
}

// ============= Magnetometer Functions =============

void ICM20948_MagWrite(uint8_t reg, uint8_t data) {
    ICM20948_SelectBank(3);
    
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_DO, data);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81);
    
    delay_ms(10);
    ICM20948_SelectBank(0);
}

uint8_t ICM20948_MagRead(uint8_t reg) {
    ICM20948_SelectBank(3);
    
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81);
    
    delay_ms(10);
    ICM20948_SelectBank(0);
    
    return ICM20948_ReadByte(ICM20948_EXT_SLV_SENS_DATA_00);
}

// ============= ICM20948 Initialization =============

uint8_t ICM20948_Init(void) {
    uint8_t whoami;
    
    delay_ms(100);
    
    // Kiểm tra WHO_AM_I
    ICM20948_SelectBank(0);
    whoami = ICM20948_ReadByte(ICM20948_WHO_AM_I);
    
    if(whoami != ICM20948_DEVICE_ID) {
        return 0;
    }
    
    // Reset device
    ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x80);
    delay_ms(100);
    
    // Wake up và chọn best clock
    ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x01);
    delay_ms(10);
    
    // Disable I2C, chỉ dùng SPI
    ICM20948_WriteByte(ICM20948_USER_CTRL, 0x10);
    delay_ms(10);
    
    // Enable Accel & Gyro
    ICM20948_WriteByte(ICM20948_PWR_MGMT_2, 0x00);
    delay_ms(10);
    
    // Cấu hình Gyro (Bank 2)
    ICM20948_SelectBank(2);
    ICM20948_WriteByte(ICM20948_GYRO_SMPLRT_DIV, 0x04);
    ICM20948_WriteByte(ICM20948_GYRO_CONFIG_1, 0x01); // ±250dps
    ICM20948_WriteByte(ICM20948_GYRO_CONFIG_2, 0x00);
    
    // Cấu hình Accel (Bank 2)
    ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
    ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_2, 0x04);
    ICM20948_WriteByte(ICM20948_ACCEL_CONFIG, 0x01); // ±2g
    ICM20948_WriteByte(ICM20948_ACCEL_CONFIG_2, 0x00);
    
    // Bật I2C Master Mode
    ICM20948_SelectBank(0);
    ICM20948_WriteByte(ICM20948_USER_CTRL, 0x20);
    delay_ms(10);
    
    // Cấu hình I2C Master (Bank 3)
    ICM20948_SelectBank(3);
    ICM20948_WriteByte(ICM20948_I2C_MST_CTRL, 0x07); // 400kHz
    delay_ms(10);
    
    // Khởi tạo Magnetometer
    ICM20948_MagWrite(AK09916_CNTL3, 0x01); // Reset
    delay_ms(100);
    
    ICM20948_MagWrite(AK09916_CNTL2, 0x08); // Continuous mode 100Hz
    delay_ms(10);
    
    // Cấu hình đọc Magnetometer tự động
    ICM20948_SelectBank(3);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, AK09916_HXL);
    ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x89); // Enable + 9 bytes
    
    ICM20948_SelectBank(0);
    delay_ms(100);
    
    return 1;
}

// ============= Data Reading Functions =============

void ICM20948_ReadAccel(ICM20948_Data *data) {
    uint8_t buffer[6];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_ACCEL_XOUT_H, buffer, 6);
    
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    data->accel_x_g = data->accel_x * ACCEL_SCALE_2G;
    data->accel_y_g = data->accel_y * ACCEL_SCALE_2G;
    data->accel_z_g = data->accel_z * ACCEL_SCALE_2G;
}

void ICM20948_ReadGyro(ICM20948_Data *data) {
    uint8_t buffer[6];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_GYRO_XOUT_H, buffer, 6);
    
    data->gyro_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->gyro_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->gyro_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    data->gyro_x_dps = data->gyro_x * GYRO_SCALE_250DPS;
    data->gyro_y_dps = data->gyro_y * GYRO_SCALE_250DPS;
    data->gyro_z_dps = data->gyro_z * GYRO_SCALE_250DPS;
}

void ICM20948_ReadMag(ICM20948_Data *data) {
    uint8_t buffer[9];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_EXT_SLV_SENS_DATA_00, buffer, 9);
    
    if(buffer[0] & 0x01) { // Data ready
        data->mag_x = (int16_t)((buffer[2] << 8) | buffer[1]);
        data->mag_y = (int16_t)((buffer[4] << 8) | buffer[3]);
        data->mag_z = (int16_t)((buffer[6] << 8) | buffer[5]);
        
        data->mag_x_ut = data->mag_x * MAG_SCALE_FACTOR;
        data->mag_y_ut = data->mag_y * MAG_SCALE_FACTOR;
        data->mag_z_ut = data->mag_z * MAG_SCALE_FACTOR;
    }
}

void ICM20948_ReadTemp(ICM20948_Data *data) {
    uint8_t buffer[2];
    
    ICM20948_SelectBank(0);
    ICM20948_ReadBytes(ICM20948_TEMP_OUT_H, buffer, 2);
    
    data->temperature = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->temp_c = (data->temperature / 333.87f) + 21.0f;
}

void ICM20948_ReadAll(ICM20948_Data *data) {
    ICM20948_ReadAccel(data);
    ICM20948_ReadGyro(data);
    ICM20948_ReadMag(data);
    ICM20948_ReadTemp(data);
}

void ICM20948_GetAngles(ICM20948_Data *data, float *roll, float *pitch) {
    *roll = atan2f(data->accel_y_g, data->accel_z_g) * 57.2958f;
    *pitch = atan2f(-data->accel_x_g, sqrtf(data->accel_y_g * data->accel_y_g + 
                                             data->accel_z_g * data->accel_z_g)) * 57.2958f;
}

float ICM20948_GetHeading(ICM20948_Data *data) {
    float heading = atan2f(data->mag_y_ut, data->mag_x_ut) * 57.2958f;
    if(heading < 0) heading += 360.0f;
    return heading;
}

// ============= Main Program =============

int main(void) {
    ICM20948_Data imu_data;
    float roll, pitch, heading;
    
    // Khởi tạo GPIO và SPI1
    GPIO_Init();
    SPI1_Init();
    
    CS_High();
    delay_ms(100);
    
    // Khởi tạo ICM20948
    if(!ICM20948_Init()) {
        // Lỗi khởi tạo
        while(1) {
            delay_ms(500);
        }
    }
    
    // Main loop
    while(1) {
        ICM20948_ReadAll(&imu_data);
        
        ICM20948_GetAngles(&imu_data, &roll, &pitch);
        heading = ICM20948_GetHeading(&imu_data);
        
        // Dữ liệu sẵn sàng sử dụng:
        // imu_data.accel_x_g, gyro_x_dps, mag_x_ut, temp_c
        // roll, pitch, heading
        
        delay_ms(100); // 10Hz
    }
}