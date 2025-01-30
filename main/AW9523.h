/**
 * @file AW9523.h
 * @brief AW9523 I2C Driver
 * @version 1.0
 * @author  alink777
 * @date   2025-01-30
 */
#pragma once

#ifndef AW9523_H
#define AW9523_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define AW9523_I2C_ADDR 0x5b               /* I2C default address, which can be modified by hardware, at this time A0 = 1, A1 = 1 */
#define AW9523_SCL_IO GPIO_NUM_18          /* GPIO number used for I2C master clock */
#define AW9523_SDA_IO GPIO_NUM_17          /* GPIO number used for I2C master data  */
#define AW9523_I2C_MASTER_NUM 0            /* I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define AW9523_I2C_MASTER_FREQ_HZ 400000   /* I2C master clock frequency */
#define AW9523_I2C_MASTER_TX_BUF_DISABLE 0 /* I2C master doesn't need buffer */
#define AW9523_I2C_MASTER_RX_BUF_DISABLE 0 /* I2C master doesn't need buffer */
#define AW9523_I2C_MASTER_TIMEOUT_MS 1000  /* I2C operation timeout in milliseconds */

/* REG ADDRESS */
typedef enum
{
    // 00H : D[7:0] -> P0_7~P0_0 Input status, 01H : D[7:0] -> P0_7~P0_0 Input status
    AW9523_REG_INPUT_P0 = 0x00, // Read-only, P0 port input status, can clear interrupts caused by P0 port when reading
    AW9523_REG_INPUT_P1 = 0x01, // Read-only, P1 port input status, can clear interrupts caused by P1 port when reading

    // 02H : D[7:0] -> P0_7~P0_0 Output status, 03H : D[7:0] -> P1_7~P1_0 Output status
    AW9523_REG_OUTPUT_P0 = 0x02, // Read/write, set the output status of P0 port, 0- output low; 1- Output high
    AW9523_REG_OUTPUT_P1 = 0x03, // Read/write, set the output status of P1 port, 0- output low; 1- Output high

    // 04H : D[7:0] -> P0_7~P0_0 in/out mode, 05H : D[7:0] -> P1_7~P1_0 in/out mode
    AW9523_REG_CONFIG_P0 = 0x04, // Read/write, set the P0 port input or output mode, 0- output; 1- input
    AW9523_REG_CONFIG_P1 = 0x05, // Read/write, set the P0 port input or output mode, 0- output; 1- Output

    // 06H : D[7:0] -> P0_7~P0_0 interrupt enable, 07H : D[7:0] -> P1_7~P1_0 interrupt enable
    AW9523_REG_INT_PO = 0x06, // Read/write, P0 interrupt enabled, 0- enabled; 1- disable
    AW9523_REG_INT_P1 = 0x07, // Read/write, P1 interrupt enabled, 0- enabled; 1- disable

    AW9523_REG_ID = 0x10, // Read-only, device ID registers, default value is 0x23

    // If D[4]=0, the P0 port is in Open-Drain mode (external pull-up resistor is required); If D[4]=1, P0 is in push-pull mode.
    // D[1:0] set I-max, I-max default is 37ma, 00: 37mA, 01: 27.75mA, 10: 18.5mA, 11: 9.25mA.
    // D[7:5], D[3:2] are left by default and must be set to 0 if changes are needed
    AW9523_REG_CTRL = 0x11, // Read/write, global control register

    // D[7:0] set Px_7 ~ Px_0 led / gpio(default) mode, 0- LED mode; 1- GPIO mode。
    AW9523_REG_MODE_P0 = 0x12, // Read/write, Port0 -> 0 LED mode; 1 GPIO mode
    AW9523_REG_MODE_P1 = 0x13, // Read/write, Port1 -> 0 LED mode; 1 GPIO mode

    // Set I-max Current, Support 256 Levels of Regulation, D[7:0]
    AW9523_REG_DIM_0 = 0x20,  // Write-Only, P1_0 LED Current Control (0 - Imax)
    AW9523_REG_DIM_1 = 0x21,  // Write-Only, P1_1 LED Current Control (0 - Imax)
    AW9523_REG_DIM_2 = 0x22,  // Write-Only, P1_2 LED Current Control (0 - Imax)
    AW9523_REG_DIM_3 = 0x23,  // Write-Only, P1_3 LED Current Control (0 - Imax)
    AW9523_REG_DIM_4 = 0x24,  // Write-Only, P0_0 LED Current Control (0 - Imax)
    AW9523_REG_DIM_5 = 0x25,  // Write-Only, P0_1 LED Current Control (0 - Imax)
    AW9523_REG_DIM_6 = 0x26,  // Write-Only, P0_2 LED Current Control (0 - Imax)
    AW9523_REG_DIM_7 = 0x27,  // Write-Only, P0_3 LED Current Control (0 - Imax)
    AW9523_REG_DIM_8 = 0x28,  // Write-Only, P0_4 LED Current Control (0 - Imax)
    AW9523_REG_DIM_9 = 0x29,  // Write-Only, P0_5 LED Current Control (0 - Imax)
    AW9523_REG_DIM_10 = 0x2A, // Write-Only, P0_6 LED Current Control (0 - Imax)
    AW9523_REG_DIM_11 = 0x2B, // Write-Only, P0_7 LED Current Control (0 - Imax)
    AW9523_REG_DIM_12 = 0x2C, // Write-Only, P1_4 LED Current Control (0 - Imax)
    AW9523_REG_DIM_13 = 0x2D, // Write-Only, P1_5 LED Current Control (0 - Imax)
    AW9523_REG_DIM_14 = 0x2E, // Write-Only, P1_6 LED Current Control (0 - Imax)
    AW9523_REG_DIM_15 = 0x2F, // Write-Only, P1_7 LED Current Control (0 - Imax)

    AW9523_REG_RST = 0x7F // Write only, set 0X00 software reset

    // 08H~0FH; 14H~1FH; 30H~7EH; 80H~FFH not available......

} AW9523_Reg_t;

/**
 * @brief AW9523 Port, 0: P0; 1: P1
 */
typedef enum
{
    AW9523_PORT_0 = 0, // PORT 0
    AW9523_PORT_1 = 1  // PORT 1
} AW9523_Port_t;

/**
 * @brief AW9523 Mode, 0: LED_Mode; 1: GPIO_Mode(default)
 */
typedef enum
{
    AW9523_MODE_LED = 0,  // LED_Mode
    AW9523_MODE_GPIO = 1, // GPIO_Mode(default)
} AW9523_Mode_t;

/**
 * @brief AW9523 Mode in/out, 0: output; 1: input
 */
typedef enum
{
    AW9523_MODE_OUT = 0, // output
    AW9523_MODE_IN = 1   // input
} AW9523_InOut_t;

/**
 * @brief AW9523 Pin Number
 */
typedef enum
{
    AW9523_PX_0 = 0,
    AW9523_PX_1,
    AW9523_PX_2,
    AW9523_PX_3,
    AW9523_PX_4,
    AW9523_PX_5,
    AW9523_PX_6,
    AW9523_PX_7
} AW9523_Pin_t;

/**
 * @brief IO output maximum current value -  37ma(default); 0x00 ~ 0x03
 */
typedef enum
{
    AW9523_CURR_37M = 0x00,    // Imax 37ma(default)
    AW9523_CURR_27_75M = 0x01, // 3/4 * Imax
    AW9523_CURR_18_5M = 0x02,  // 2/4 * Imax
    AW9523_CURR_13_875M = 0x03 // 1/4 * Imax
} AW9523_Current_t;

/**
 * @brief Initialize the AW9523 I2C master
 */
esp_err_t aw9523_i2c_init(void);

/**
 * @brief Read the AW9523 register
 */
esp_err_t aw9523_reg_read(uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write 1 byte to the specified AW9523 register
 */
esp_err_t aw9523_reg_write_byte(uint8_t reg_addr, uint8_t data);

/**
 * @brief Read the port input level
 * @param port
 * @return uint8_t Px_7~Px_0
 */
uint8_t aw9523_read_level(AW9523_Port_t port);

/**
 * @brief Set the port output level
 * @param port
 * @param uint8_t value_byte Px_7~Px_0
 */
void aw9523_set_port_level(AW9523_Port_t port, uint8_t value_byte);

/**
 * @brief Set the port_pin output level
 * @param port
 * @param pin_num
 * @param value_bit 0 or 1
 */
void aw9523_set_pin_level(AW9523_Port_t port, AW9523_Pin_t pin_num, uint8_t value_bit);

/**
 * @brief Set the port input / output mode
 * @param port
 * @param inout_mode_byte D[7:0] set Px_7 ~ Px_0 in/out mode, 0-output; 1-input
 */
void aw9523_set_port_inout(AW9523_Port_t port, uint8_t inout_mode_byte);

/**
 * @brief Set the port_pin input / output mode
 * @param port
 * @param pin_num
 * @param inout_mode_bit
 */
void aw9523_set_pin_inout(AW9523_Port_t port, AW9523_Pin_t pin_num, AW9523_InOut_t inout_mode_bit);

/**
 * @brief If D[4]=0, the P0 port is in Open-Drain mode (external pull-up resistor is required); If D[4]=1, P0 is in push-pull mode.
    D[1:0] port output maximum current value, I-max default is 37ma, 00: 37mA, 01: 27.75mA, 10: 18.5mA, 11: 9.25mA.
    D[7:5], D[3:2] are left by default and must be set to 0 if changes are needed
 * @param ctrl_cmd_byte
 */
void aw9523_set_ctl(uint8_t ctrl_cmd_byte);

/**
 * @brief Sets the LED maximum current
 * @param current
 */
void aw9523_set_led_max_current(AW9523_Current_t current);

/**
 * @brief Settings Specify the GPIO or LED mode of the port
 * @param port
 * @param mode_byte D[7:0] set Px_7 ~ Px_0, 0: LED mode; 1: GPIO mode(default)
 */
void aw9523_set_port_gpio_or_led(AW9523_Port_t port, uint8_t mode_byte);

/**
 * @brief Settings Specify the GPIO or LED mode of the port_pin
 * @param port
 * @param pin_num
 * @param mode_bit 0: LED mode; 1: GPIO mode(default)
 */
void aw9523_set_pin_gpio_or_led(AW9523_Port_t port, AW9523_Pin_t pin_num, AW9523_Mode_t mode_bit);

/**
 * @brief Set the LED duty of the specified port individually
 * @param port
 * @param duty_byte  0x00 ~ 0xff
 */
void aw9523_set_port_duty(AW9523_Port_t port, uint8_t duty_byte);

/**
 * @brief Set the LED duty of the specified port_pin individually
 * @param port
 * @param pin_num
 * @param duty_byte  0x00 ~ 0xff
 */
void aw9523_set_pin_duty(AW9523_Port_t port, AW9523_Pin_t pin_num, uint8_t duty_byte);

/**
 * @brief Reset AW9523
 */
void aw9523_reset(void);

/**
 * @brief AW9523 Demo
 */
void aw9523_Demo(void);

#endif