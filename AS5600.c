/**
 * @file AS5600.c
 * @author JanG175
 * @brief ESP IDF component for the AS5600 magnetic rotary encoder
 * 
 * @copyright Apache 2.0
 */

#include <stdio.h>
#include "AS5600.h"

static const char *TAG = "AS5600";


/**
 * @brief send data to AS5600 register
 * 
 * @param as struct with AS5600 parameters
 * @param reg register to write to
 * @param data pointer data to write
 * @param data_len length of data to write
*/
static void i2c_send(AS5600_t as, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, reg, ACK_EN);
    i2c_master_write(cmd, data, data_len, ACK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(as.i2c_port, cmd, AS5600_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}


/**
 * @brief read data from AS5600 register
 * 
 * @param as struct with AS5600 parameters
 * @param reg register to read from
 * @param data pointer to data to read
 * @param data_len length of data to read
*/
static void i2c_read(AS5600_t as, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_WRITE, AS5600_ADDRESS);
    i2c_master_write_byte(cmd, reg, ACK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_READ, ACK_EN);

    for (uint32_t i = 0; i < data_len - 1; i++)
        i2c_master_read_byte(cmd, &data[i], ACK);
    i2c_master_read_byte(cmd, &data[data_len - 1], NACK);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(as.i2c_port, cmd, AS5600_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}


/**
 * @brief initialize the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_init(AS5600_t as)
{
    if (as.i2c_freq > AS5600_MAX_FREQ)
    {
        as.i2c_freq = AS5600_MAX_FREQ;
        ESP_LOGW(TAG, "I2C frequency too high, set to max value (%d Hz)", AS5600_MAX_FREQ);
    }

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = as.sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = as.scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = as.i2c_freq,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(as.i2c_port, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(as.i2c_port, i2c_config.mode, 0, 0, 0));
}


/**
 * @brief deinitialize the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_deinit(AS5600_t as)
{
    ESP_ERROR_CHECK(i2c_driver_delete(as.i2c_port));
}


/**
 * @brief get the angle from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return angle in degrees
*/
float AS5600_get_angle(AS5600_t as)
{
    uint8_t data[2];
    i2c_read(as, ANGLE_REG, data, sizeof(data));

    float angle = (float)(((int32_t)data[0] << 8) | (int32_t)data[1]);

    angle = angle * 360.0f / 4096.0f;

    return angle;
}


/**
 * @brief get the raw angle from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return raw angle in degrees
*/
float AS5600_get_raw_angle(AS5600_t as)
{
    uint8_t data[2];
    i2c_read(as, RAW_ANGLE_REG, data, sizeof(data));

    data[0] &= 0b00001111;

    float raw_angle = (float)(((int32_t)data[0] << 8) | (int32_t)data[1]);
    raw_angle = raw_angle * 360.0f / 4096.0f;

    return raw_angle;
}


/**
 * @brief print the magnet status from the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_get_magnet_status(AS5600_t as)
{
    uint8_t data;

    i2c_read(as, STATUS_REG, &data, sizeof(data));

    if ((data & 0b00001000) == 0b00001000)
        ESP_LOGW(TAG, "magnet too strong");
    else if ((data & 0b00010000) == 0b00010000)
        ESP_LOGW(TAG, "magnet too weak");

    if ((data & 0b00100000) == 0b00100000)
        ESP_LOGI(TAG, "magnet in place");
    else
        ESP_LOGE(TAG, "magnet missing");
}


/**
 * @brief read the AGC register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return AGC value
*/
uint8_t AS5600_get_agc(AS5600_t as)
{
    uint8_t data;

    i2c_read(as, AGC_REG, &data, sizeof(data));

    return data;
}


/**
 * @brief read the magnitude register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return magnitude value
*/
uint16_t AS5600_get_magnitude(AS5600_t as)
{
    uint8_t data[2];

    i2c_read(as, MAGNITUDE_REG, data, sizeof(data));

    data[0] &= 0b00001111;

    uint16_t magnitude = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return magnitude;
}


/**
 * @brief read the ZMCO register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return ZMCO value
*/
uint8_t AS5600_get_ZMCO(AS5600_t as)
{
    uint8_t data;

    i2c_read(as, ZMCO_REG, &data, sizeof(data));

    data &= 0b00000011;

    return data;
}


/**
 * @brief read the ZPOS register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return ZPOS value
*/
uint16_t AS5600_get_ZPOS(AS5600_t as)
{
    uint8_t data[2];

    i2c_read(as, ZPOS_REG, data, sizeof(data));

    data[0] &= 0b00001111;

    uint16_t zpos = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return zpos;
}


/**
 * @brief read the MPOS register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return MPOS value
*/
uint16_t AS5600_get_MPOS(AS5600_t as)
{
    uint8_t data[2];

    i2c_read(as, MPOS_REG, data, sizeof(data));

    data[0] &= 0b00001111;

    uint16_t mpos = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return mpos;
}


/**
 * @brief read the MANG register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * 
 * @return MANG value
*/
uint16_t AS5600_get_MANG(AS5600_t as)
{
    uint8_t data[2];

    i2c_read(as, MANG_REG, data, sizeof(data));

    data[0] &= 0b00001111;

    uint16_t mang = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return mang;
}


/**
 * @brief set the CONF register from the AS5600
 * 
 * @param as struct with AS5600 parameters
 * @param value value to set
 * 
 * @return CONF value
*/
uint16_t AS5600_get_CONF(AS5600_t as)
{
    uint8_t data[2];

    i2c_read(as, CONF_REG, data, sizeof(data));

    data[0] &= 0b00111111;

    uint16_t conf = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

    return conf;
}


/**
 * @brief print the CONF register from the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_print_CONF(AS5600_t as)
{
    uint16_t conf = AS5600_get_CONF(as);

    uint8_t pm =   (uint8_t)((conf & 0b0000000000000011) >> 0);
    uint8_t hyst = (uint8_t)((conf & 0b0000000000001100) >> 2);
    uint8_t outs = (uint8_t)((conf & 0b0000000000110000) >> 4);
    uint8_t pwmf = (uint8_t)((conf & 0b0000000011000000) >> 6);
    uint8_t sf =   (uint8_t)((conf & 0b0000001100000000) >> 8);
    uint8_t fth =  (uint8_t)((conf & 0b0001110000000000) >> 10);
    uint8_t wd =   (uint8_t)((conf & 0b0010000000000000) >> 13);

    ESP_LOGI(TAG, "PM: %u", pm);
    ESP_LOGI(TAG, "HYST: %u", hyst);
    ESP_LOGI(TAG, "OUTS: %u", outs);
    ESP_LOGI(TAG, "PWMF: %u", pwmf);
    ESP_LOGI(TAG, "SF: %u", sf);
    ESP_LOGI(TAG, "FTH: %u", fth);
    ESP_LOGI(TAG, "WD: %u", wd);
}


/**
 * @brief set the CONF register of the AS5600
 * 
 * @param as struct with AS5600 parameters
 * @param pm power mode
 * @param hyst hysteresis
 * @param outs output stage
 * @param pwmf PWM frequency
 * @param sf slow filter
 * @param fth fast filter threshold
 * @param wd watchdog
*/
void AS5600_set_CONF(AS5600_t as, uint8_t pm, uint8_t hyst, uint8_t outs, uint8_t pwmf, uint8_t sf, uint8_t fth, uint8_t wd)
{
    uint16_t conf = ((uint16_t)pm << 0) | ((uint16_t)hyst << 2) | ((uint16_t)outs << 4) | ((uint16_t)pwmf << 6) | 
                        ((uint16_t)sf << 8) | ((uint16_t)fth << 10) | ((uint16_t)wd << 13);

    // read current CONF register
    uint8_t data[2];
    i2c_read(as, CONF_REG, data, sizeof(data));
    uint16_t cur_conf = (uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1];

    printf("cur_conf: %x\n", cur_conf);

    // bits 15 and 14 are reserved
    if ((cur_conf & (1 << 15)) == 0)
        conf &= ~(1 << 15); // set bit to 0
    else
        conf |= (1 << 15); // set bit to 1

    if ((cur_conf & (1 << 14)) == 0)
        conf &= ~(1 << 14); // set bit to 0
    else
        conf |= (1 << 14); // set bit to 1

    printf("conf: %x\n", conf);

    // set data to write
    data[0] = (uint8_t)(conf >> 8);
    data[1] = (uint8_t)(conf & 0xFF);

    i2c_send(as, CONF_REG, data, sizeof(data));
}


/**
 * @brief set the ZPOS register of the AS5600
 * 
 * @param as struct with AS5600 parameters
 * @param zpos value to set
*/
void AS5600_set_ZPOS(AS5600_t as, uint16_t zpos)
{
    // read current ZPOS register
    uint8_t data[2];
    i2c_read(as, ZPOS_REG, data, sizeof(data));
    uint16_t cur_zpos = (uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1];

    // bits 15 to 12 are reserved
    for (uint8_t i = 15; i >= 12; i--)
    {
        if ((cur_zpos & (1 << i)) == 0)
            zpos &= ~(1 << i); // set bit to 0
        else
            zpos |= (1 << i); // set bit to 1
    }

    // set data to write
    data[0] = (uint8_t)(zpos >> 8);
    data[1] = (uint8_t)(zpos & 0xFF);

    i2c_send(as, ZPOS_REG, data, sizeof(data));
}


/**
 * @brief set the MPOS register of the AS5600
 * 
 * @param as struct with AS5600 parameters
 * @param mpos value to set
*/
void AS5600_set_MPOS(AS5600_t as, uint16_t mpos)
{
    // read current MPOS register
    uint8_t data[2];
    i2c_read(as, MPOS_REG, data, sizeof(data));
    uint16_t cur_mpos = (uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1];

    // bits 15 to 12 are reserved
    for (uint8_t i = 15; i >= 12; i--)
    {
        if ((cur_mpos & (1 << i)) == 0)
            mpos &= ~(1 << i); // set bit to 0
        else
            mpos |= (1 << i); // set bit to 1
    }

    // set data to write
    data[0] = (uint8_t)(mpos >> 8);
    data[1] = (uint8_t)(mpos & 0xFF);

    i2c_send(as, MPOS_REG, data, sizeof(data));
}


/**
 * @brief set the MANG register of the AS5600
 * 
 * @param as struct with AS5600 parameters
 * @param mang value to set
*/
void AS5600_set_MANG(AS5600_t as, uint16_t mang)
{
    // read current MANG register
    uint8_t data[2];
    i2c_read(as, MANG_REG, data, sizeof(data));
    uint16_t cur_mang = (uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1];

    // bits 15 to 12 are reserved
    for (uint8_t i = 15; i >= 12; i--)
    {
        if ((cur_mang & (1 << i)) == 0)
            mang &= ~(1 << i); // set bit to 0
        else
            mang |= (1 << i); // set bit to 1
    }

    // set data to write
    data[0] = (uint8_t)(mang >> 8);
    data[1] = (uint8_t)(mang & 0xFF);

    i2c_send(as, MANG_REG, data, sizeof(data));
}


/**
 * @brief burn the angle to the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_burn_angle(AS5600_t as)
{
    uint8_t data = BURN_ANGLE;

    i2c_send(as, BURN_REG, &data, sizeof(data));
}


/**
 * @brief burn the setting to the AS5600
 * 
 * @param as struct with AS5600 parameters
*/
void AS5600_burn_setting(AS5600_t as)
{
    uint8_t data = BURN_SET;

    i2c_send(as, BURN_REG, &data, sizeof(data));
}