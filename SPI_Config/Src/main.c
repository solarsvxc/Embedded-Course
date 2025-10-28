#include "gpio_config.h"
#include "systick.h"
#include "spi.h"
#include "icm20948.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define GPIOBEN         (1U<<1)
#define GPIOAEN         (1U<<0)

#define LED_NORTH_PIN   (1U<<0)  /* PB0 - Bắc */
#define LED_EAST_PIN    (1U<<1)  /* PB1 - Đông */
#define LED_SOUTH_PIN   (1U<<2)  /* PB2 - Nam */
#define LED_WEST_PIN    (1U<<3)  /* PB3 - Tây */
/* Góc định hướng (±45°) */
#define NORTH_MIN       315.0f
#define NORTH_MAX       45.0f
#define EAST_MIN        45.0f
#define EAST_MAX        135.0f
#define SOUTH_MIN       135.0f
#define SOUTH_MAX       225.0f
#define WEST_MIN        225.0f
#define WEST_MAX        315.0f
/* Filtering */
#define ALPHA           0.2f  


/*******************************************************************************
 * Variables
 ******************************************************************************/
static float filtered_heading = 0.0f;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void  direction_leds_init(void);
void  turn_off_all_leds(void);
void  display_direction(float heading);
float low_pass_filter(float new_value, float old_value);
void  calibration_mode(ICM20948_Data *data);


/*******************************************************************************
 * Code
 ******************************************************************************/
int main(void)
{
    ICM20948_Data imu_data = {0};


    float heading;
    uint8_t init_status;


    led_init();
    led_off();
    direction_leds_init();
    button_init();


    spi1_init();
    spi1_config();


    cs_disable();


    delay_ms(100);


    /* Khởi tạo ICM20948 */
    led_on();
    init_status = ICM20948_Init();


    /* if init error ICM20948  */
    if (!init_status)
        {
            led_off();


            while(1) {
                led_toggle();
                delay_ms(100);
            }
        } else
        {
            /* DO NOTTHING */
        }
    


        /* if init success */
    led_off();
    delay_ms(500);


    for(uint8_t i = 0; i < 3; i++)
    {
        led_on();
        delay_ms(200);
        led_off();
        delay_ms(200);
    }


    /* check button to get in calibration mode */
    if(get_btn_state()) {
        led_on();
        delay_ms(1000);
        calibration_mode(&imu_data);
        led_off();
    } else
    {
        /* DO NOTTHING */
    }
   


    /* read first value */
    ICM20948_ReadMag(&imu_data);
    filtered_heading = ICM20948_GetHeading(&imu_data);


    while(1)
    {
        /* Đọc dữ liệu */
        ICM20948_ReadMag(&imu_data);


        /* Tính heading */
        heading = ICM20948_GetHeading(&imu_data);


        /* Lọc nhiễu */
        filtered_heading = low_pass_filter(heading, filtered_heading);


        /* Hiển thị hướng */
        display_direction(filtered_heading);


        static uint16_t toggle_counter = 0;
 if(++toggle_counter >= 50) {    
            led_toggle();
            toggle_counter = 0;
        }


        delay_ms(100);


        if(get_btn_state())
        {
            delay_ms(50); /* Debounce */


            if(get_btn_state())
            {
                while(get_btn_state());    
                calibration_mode(&imu_data);
                /* Reset filtered heading */
                ICM20948_ReadMag(&imu_data);
                filtered_heading = ICM20948_GetHeading(&imu_data);
            }
        }
    }
    return 0;
}


void direction_leds_init(void)
{
    RCC->AHB1ENR |= GPIOBEN;


    /* PB0 as output */
    GPIOB->MODER |=  (1U<<0);
    GPIOB->MODER &= ~(1U<<1);
    /* PB1 as output */
    GPIOB->MODER |=  (1U<<2);
    GPIOB->MODER &= ~(1U<<3);
    /* PB2 as output */
    GPIOB->MODER |=  (1U<<4);
    GPIOB->MODER &= ~(1U<<5);
    /* PB3 as output */
    GPIOB->MODER |=  (1U<<6);
    GPIOB->MODER &= ~(1U<<7);


    /* Set output speed to high */
    GPIOB->OSPEEDR |= (3U<<0);
    GPIOB->OSPEEDR |= (3U<<2);
    GPIOB->OSPEEDR |= (3U<<4);
    GPIOB->OSPEEDR |= (3U<<6);


    turn_off_all_leds();
}


void turn_off_all_leds(void)
{
    GPIOB->ODR &= ~(LED_NORTH_PIN | LED_EAST_PIN | LED_SOUTH_PIN | LED_WEST_PIN);
}


void display_direction(float heading)
{
    turn_off_all_leds();


    if (heading >= NORTH_MIN || heading < NORTH_MAX) {
        GPIOB->ODR |= LED_NORTH_PIN;
    }
    else if (heading >= EAST_MIN && heading < EAST_MAX) {
        GPIOB->ODR |= LED_EAST_PIN;
    }
    else if (heading >= SOUTH_MIN && heading < SOUTH_MAX) {
        GPIOB->ODR |= LED_SOUTH_PIN;
    }
    else if (heading >= WEST_MIN && heading < WEST_MAX) {
        GPIOB->ODR |= LED_WEST_PIN;
    }
}


/**
 * @brief Low-pass filter để làm mượt dữ liệu
 */
float low_pass_filter(float new_value, float old_value)
{
    /* Xử lý trường hợp đặc biệt khi góc qua 0°/360° */
    float diff = new_value - old_value;


    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }


    float filtered = old_value + ALPHA * diff;


    /* Normalize về 0-360° */
    if (filtered < 0.0f) {
        filtered += 360.0f;
    } else if (filtered >= 360.0f) {
        filtered -= 360.0f;
    }


    return filtered;
}


/**
 * @brief Chế độ calibration - xoay thiết bị theo hình số 8
 */
void calibration_mode(ICM20948_Data *data)
{
    /* Nhấp nháy tất cả LED để báo hiệu calibration */
    for(uint8_t i = 0; i < 6; i++) {
        GPIOB->ODR |= (LED_NORTH_PIN | LED_EAST_PIN | LED_SOUTH_PIN | LED_WEST_PIN);
   delay_ms(200);
        turn_off_all_leds();
        delay_ms(200);
    }


    /* Bắt đầu calibration - xoay thiết bị theo hình số 8 */
    /* LED quay vòng để báo hiệu đang calibrate */
    for(uint16_t i = 0; i < 300; i++) {
        if(i % 4 == 0) GPIOB->ODR = LED_NORTH_PIN;
        else if(i % 4 == 1) GPIOB->ODR = LED_EAST_PIN;
        else if(i % 4 == 2) GPIOB->ODR = LED_SOUTH_PIN;
        else GPIOB->ODR = LED_WEST_PIN;


        delay_ms(10);
    }


    turn_off_all_leds();


    ICM20948_CalibrateMag(data, 300);


    /* Nhấp nháy nhanh báo hiệu hoàn tất */
    for(uint8_t i = 0; i < 10; i++) {
        GPIOB->ODR |= (LED_NORTH_PIN | LED_EAST_PIN | LED_SOUTH_PIN | LED_WEST_PIN);
        delay_ms(100);
        turn_off_all_leds();
        delay_ms(100);
    }
}