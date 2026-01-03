#include "icm20948.h"
#include "spi.h"
#include "systick.h"

/* -------------------------------------------------------------------------- */
/* DEFINITIONS & SENSITIVITY CONSTANTS                                        */
/* -------------------------------------------------------------------------- */

#define SPI_READ_FLAG   0x80
#define SPI_WRITE_FLAG  0x7F

#define ACCEL_SENSITIVITY_FSR_4G    8192.0f
#define GYRO_SENSITIVITY_FSR_2000   16.4f
#define MAG_SENSITIVITY             0.15f
#define TEMP_SENSITIVITY            333.87f
#define TEMP_OFFSET_C               21.0f

/* -------------------------------------------------------------------------- */
/* PRIVATE FUNCTION PROTOTYPES                                                */
/* -------------------------------------------------------------------------- */

static void icm20948_set_bank(uint8_t bank);
static void icm20948_write_byte(uint8_t reg_address, uint8_t data);
static uint8_t icm20948_read_byte(uint8_t reg_address);
static void icm20948_read_burst(uint8_t reg_address, uint8_t *data, uint32_t size);
static void icm20948_write_mag_reg(uint8_t reg_address, uint8_t data);

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS                                                           */
/* -------------------------------------------------------------------------- */

int8_t icm20948_init(void) 
{
    uint8_t who_am_i = 0;

    icm20948_set_bank(ICM20948_BANK_0);

    /* 1. Reset thiết bị */
    icm20948_write_byte(ICM20948_REG_B0_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET_BIT);
    
    delay_ms(100); 

    who_am_i = icm20948_read_byte(ICM20948_REG_B0_WHO_AM_I);

    if (who_am_i != ICM20948_WHO_AM_I_VAL)
    {
        return -1;
    }
    
    /* 2. Cấu hình cơ bản (Clock, Bật cảm biến) */
    icm20948_write_byte(ICM20948_REG_B0_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_BEST);
    icm20948_write_byte(ICM20948_REG_B0_PWR_MGMT_2, 0x00);

    /* 3. Cấu hình Accel & Gyro */
    icm20948_set_bank(ICM20948_BANK_2);

    /* ACCEL: 0x0B = ±4g (01 << 1) | DLPF Bật (1) | DLPFCFG=1 (001 << 3) */
    icm20948_write_byte(ICM20948_REG_B2_ACCEL_CONFIG, 0x0B); 
    icm20948_write_byte(ICM20948_REG_B2_ACCEL_SMPLRT_DIV_1, 0x00);
    icm20948_write_byte(ICM20948_REG_B2_ACCEL_SMPLRT_DIV_2, 10); /*<! ODR ~102Hz */

    /* GYRO: 0x0B = ±2000dps (11 << 1) | DLPF Bật (1) | DLPFCFG=1 (001 << 3) */
    icm20948_write_byte(ICM20948_REG_B2_GYRO_CONFIG_1, 0x0B);
    icm20948_write_byte(ICM20948_REG_B2_GYRO_SMPLRT_DIV, 10); /*<! ODR ~102Hz */

    /* 4. Cấu hình I2C Master (để đọc từ kế) */
    icm20948_set_bank(ICM20948_BANK_0);
    icm20948_write_byte(ICM20948_REG_B0_USER_CTRL, USER_CTRL_I2C_MST_EN_BIT);

    icm20948_set_bank(ICM20948_BANK_3);
    icm20948_write_byte(ICM20948_REG_B3_I2C_MST_CTRL, I2C_MST_CTRL_CLK_400KHZ);

    /* 5. Cấu hình Từ kế (dùng SLV4 để GHI) */
    icm20948_write_mag_reg(AK09916_REG_CNTL3, AK09916_CNTL3_SRST_BIT);
    delay_ms(100);

    icm20948_write_mag_reg(AK09916_REG_CNTL2, AK09916_CNTL2_MODE_CONT_100HZ);
    delay_ms(10); 

    /* 6. Cấu hình I2C SLV0 (để TỰ ĐỘNG ĐỌC từ kế) */
    icm20948_set_bank(ICM20948_BANK_3);
    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_SLV_ADDR_READ_BIT);
    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV0_REG, AK09916_REG_HXL);
    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV0_CTRL, I2C_SLV_CTRL_EN_BIT | 8); /*<! Bật và đọc 8 byte */
    
    icm20948_set_bank(ICM20948_BANK_0);

    return 0;
}

void icm20948_read_all_raw(icm20948_raw_data_t *p_data_out)
{
    uint8_t data_buffer[14];

    icm20948_set_bank(ICM20948_BANK_0);

    icm20948_read_burst(ICM20948_REG_B0_ACCEL_XOUT_H, data_buffer, 14);

    p_data_out->accel_x = (int16_t)((data_buffer[0] << 8) | data_buffer[1]);
    p_data_out->accel_y = (int16_t)((data_buffer[2] << 8) | data_buffer[3]);
    p_data_out->accel_z = (int16_t)((data_buffer[4] << 8) | data_buffer[5]);
    
    p_data_out->gyro_x  = (int16_t)((data_buffer[6] << 8) | data_buffer[7]);
    p_data_out->gyro_y  = (int16_t)((data_buffer[8] << 8) | data_buffer[9]);
    p_data_out->gyro_z  = (int16_t)((data_buffer[10] << 8) | data_buffer[11]);

    p_data_out->temp = (int16_t)((data_buffer[12] << 8) | data_buffer[13]);
}

void icm20948_read_mag_raw(icm20948_raw_data_t *p_data_out)
{
    uint8_t data_mag_buffer[8];

    icm20948_set_bank(ICM20948_BANK_0);

    icm20948_read_burst(ICM20948_REG_B0_EXT_SLV_SENS_DATA_00, data_mag_buffer, 8);

    /* (AK09916 là Little Endian) */
    p_data_out->mag_x = (int16_t)((data_mag_buffer[1] << 8) | data_mag_buffer[0]);
    p_data_out->mag_y = (int16_t)((data_mag_buffer[3] << 8) | data_mag_buffer[2]);
    p_data_out->mag_z = (int16_t)((data_mag_buffer[5] << 8) | data_mag_buffer[4]);
}

void icm20948_convert_to_scaled(icm20948_raw_data_t *p_raw_data, icm20948_scaled_data_t *p_scaled_data)
{
    p_scaled_data->accel_x_g = ((float)p_raw_data->accel_x / ACCEL_SENSITIVITY_FSR_4G);
    p_scaled_data->accel_y_g = ((float)p_raw_data->accel_y / ACCEL_SENSITIVITY_FSR_4G);
    p_scaled_data->accel_z_g = ((float)p_raw_data->accel_z / ACCEL_SENSITIVITY_FSR_4G);
    
    p_scaled_data->gyro_x_dps = ((float)p_raw_data->gyro_x / GYRO_SENSITIVITY_FSR_2000);
    p_scaled_data->gyro_y_dps = ((float)p_raw_data->gyro_y / GYRO_SENSITIVITY_FSR_2000);
    p_scaled_data->gyro_z_dps = ((float)p_raw_data->gyro_z / GYRO_SENSITIVITY_FSR_2000);
    
    p_scaled_data->mag_x_ut = ((float)p_raw_data->mag_x * MAG_SENSITIVITY);
    p_scaled_data->mag_y_ut = ((float)p_raw_data->mag_y * MAG_SENSITIVITY);
    p_scaled_data->mag_z_ut = ((float)p_raw_data->mag_z * MAG_SENSITIVITY);
    
    p_scaled_data->temp_c = (((float)p_raw_data->temp / TEMP_SENSITIVITY) + TEMP_OFFSET_C);
}

/* -------------------------------------------------------------------------- */
/* PRIVATE (STATIC) FUNCTIONS                                                 */
/* -------------------------------------------------------------------------- */
static void icm20948_write_mag_reg(uint8_t reg_address, uint8_t data)
{
    icm20948_set_bank(ICM20948_BANK_3);
    
    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV4_ADDR, AK09916_I2C_ADDR);

    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV4_REG, reg_address);

    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV4_DO, data);
    icm20948_write_byte(ICM20948_REG_B3_I2C_SLV4_CTRL, 0x80); /*<! Kích hoạt (EN) */
    
    delay_ms(1); 
}

static void icm20948_read_burst(uint8_t reg_address, uint8_t *data, uint32_t size)
{
    uint8_t read_reg_flag;

    read_reg_flag = reg_address | SPI_READ_FLAG;

    cs_enable();
    spi1_transmit(&read_reg_flag, 1);
    spi1_receive(data, size); 
    cs_disable();
}

static void icm20948_write_byte(uint8_t reg_address, uint8_t data)
{
    uint8_t tx_buffer[2];

    tx_buffer[0] = reg_address & SPI_WRITE_FLAG;
    tx_buffer[1] = data;

    cs_enable();
    spi1_transmit(tx_buffer, 2); 
    
    cs_disable();
}

static uint8_t icm20948_read_byte(uint8_t reg_address)
{
    uint8_t rxdata;
    uint8_t read_flag;

    read_flag = reg_address | SPI_READ_FLAG;

    cs_enable();
    spi1_transmit(&read_flag, 1);
    spi1_receive(&rxdata, 1);
    cs_disable();

    return rxdata;
}

static void icm20948_set_bank(uint8_t bank)
{
    icm20948_write_byte(ICM20948_REG_BANK_SEL, bank);
}
