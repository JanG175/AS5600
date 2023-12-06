/**
 * @file AS5600.h
 * @author JanG175
 * @brief ESP IDF component for the AS5600 magnetic rotary encoder
 * 
 * @copyright Apache 2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define AS5600_MAX_FREQ 1000000
#define ACK_EN          true
#define ACK_DIS         false
#define ACK             0x00
#define NACK            0x01
#define AS5600_TIMEOUT  (100 / portTICK_PERIOD_MS)

// device address
#define AS5600_ADDRESS  0x36

// configuration registers
#define ZMCO_REG        0x00
#define ZPOS_REG        0x01 // start position
#define MPOS_REG        0x03 // stop position
#define MANG_REG        0x05 // maximum angle
#define CONF_REG        0x07

// output registers
#define RAW_ANGLE_REG   0x0C
#define ANGLE_REG       0x0E

// status registers
#define STATUS_REG      0x0B
#define AGC_REG         0x1A
#define MAGNITUDE_REG   0x1B

// burn commands
#define BURN_REG        0xFF 
#define BURN_ANGLE      0x80
#define BURN_SET        0x40

// power mode
#define PM_NOM          0x00
#define PM_LPM1         0x01
#define PM_LPM2         0x02
#define PM_LPM3         0x03

// hysterisis
#define HYST_OFF        0x00
#define HYST_1LSB       0x01
#define HYST_2LSB       0x02
#define HYST_3LSB       0x03

// output stage
#define OUTS_ANAL_FULL  0x00
#define OUTS_ANAL_RED   0x01
#define OUTS_DIG        0x02

// PWM frequency
#define PWMF_115HZ      0x00
#define PWMF_230HZ      0x01
#define PWMF_460HZ      0x02
#define PWMF_920HZ      0x03

// slow filter
#define SF_16X          0x00
#define SF_8X           0x01
#define SF_4X           0x02
#define SF_2X           0x03

// fast filter threshold
#define FTH_SLOW_ONLY   0x00
#define FTH_6LSB        0x01
#define FTH_7LSB        0x02
#define FTH_9LSB        0x03
#define FTH_18LSB       0x04
#define FTH_21LSB       0x05
#define FTH_24LSB       0x06
#define FTH_27LSB       0x07

// watchdog
#define WD_OFF          0x00
#define WD_ON           0x01

// struct for AS5600 parameters
typedef struct AS5600_t
{
    i2c_port_t i2c_port;
    uint32_t i2c_freq;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
} AS5600_t;


void AS5600_init(AS5600_t as);

void AS5600_deinit(AS5600_t as);

float AS5600_get_angle(AS5600_t as);

float AS5600_get_raw_angle(AS5600_t as);

void AS5600_get_magnet_status(AS5600_t as);

uint8_t AS5600_get_agc(AS5600_t as);

uint16_t AS5600_get_magnitude(AS5600_t as);

uint8_t AS5600_get_ZMCO(AS5600_t as);

uint16_t AS5600_get_ZPOS(AS5600_t as);

uint16_t AS5600_get_MANG(AS5600_t as);

uint16_t AS5600_get_CONF(AS5600_t as);

void AS5600_print_CONF(AS5600_t as);

void AS5600_set_CONF(AS5600_t as, uint8_t pm, uint8_t hyst, uint8_t outs, uint8_t pwmf, uint8_t sf, uint8_t fth, uint8_t wd);

void AS5600_set_ZPOS(AS5600_t as, uint16_t zpos);

void AS5600_set_MPOS(AS5600_t as, uint16_t mpos);

void AS5600_set_MANG(AS5600_t as, uint16_t mang);

void AS5600_burn_angle(AS5600_t as);

void AS5600_burn_setting(AS5600_t as);