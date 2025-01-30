#include "AW9523.h"
static const char *TAG = "AW9523";

/**
 * @brief Initialize the AW9523 I2C master
 */
esp_err_t aw9523_i2c_init(void)
{
    int i2c_master_port = AW9523_I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AW9523_SDA_IO,
        .scl_io_num = AW9523_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AW9523_I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, AW9523_I2C_MASTER_RX_BUF_DISABLE, AW9523_I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read the AW9523 register
 */
esp_err_t aw9523_reg_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(AW9523_I2C_MASTER_NUM, AW9523_I2C_ADDR, &reg_addr, 1, data, len,
                                                 AW9523_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

/**
 * @brief Write 1 byte to the specified AW9523 register
 */
esp_err_t aw9523_reg_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_write_to_device(AW9523_I2C_MASTER_NUM, AW9523_I2C_ADDR, write_buf, sizeof(write_buf),
                                               AW9523_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

/**
 * @brief Read the port input level
 * @param port
 * @return uint8_t Px_7~Px_0
 */
uint8_t aw9523_read_level(AW9523_Port_t port)
{
    uint8_t data = 0x00;
    esp_err_t ret = aw9523_reg_read((port == AW9523_PORT_0) ? AW9523_REG_INPUT_P0 : AW9523_REG_INPUT_P1, &data, sizeof(data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Read level failed");
    }
    return data;
}

/**
 * @brief Set the port output level
 * @param port
 * @param uint8_t value_byte Px_7~Px_0
 */
void aw9523_set_port_level(AW9523_Port_t port, uint8_t value_byte)
{
    esp_err_t ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1, value_byte);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set level failed");
        return;
    }
}

/**
 * @brief Set the port_pin output level
 * @param port
 * @param pin_num
 * @param value_bit 0 or 1
 */
void aw9523_set_pin_level(AW9523_Port_t port, AW9523_Pin_t pin_num, uint8_t value_bit)
{
    uint8_t port_level;
    esp_err_t ret = aw9523_reg_read((port == AW9523_PORT_0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1, &port_level, sizeof(port_level));
    if (value_bit == 1)
    {
        port_level |= (1 << pin_num);
    }
    else
    {
        port_level &= ~(1 << pin_num);
    }
    ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1, port_level);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set pin level failed");
        return;
    }
}

/**
 * @brief Set the port input / output mode
 * @param port
 * @param inout_mode_byte D[7:0] set Px_7 ~ Px_0 in/out mode, 0-output; 1-input
 */
void aw9523_set_port_inout(AW9523_Port_t port, uint8_t inout_mode_byte)
{
    esp_err_t ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_CONFIG_P0 : AW9523_REG_CONFIG_P1, inout_mode_byte);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set port inout failed");
        return;
    }
}

/**
 * @brief Set the port_pin input / output mode
 * @param port
 * @param pin_num
 * @param inout_mode_bit
 */
void aw9523_set_pin_inout(AW9523_Port_t port, AW9523_Pin_t pin_num, AW9523_InOut_t inout_mode_bit)
{
    uint8_t port_inout_mode;
    esp_err_t ret = aw9523_reg_read((port == AW9523_PORT_0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1, &port_inout_mode, sizeof(port_inout_mode));
    if (inout_mode_bit == 1)
    {
        port_inout_mode |= (1 << pin_num);
    }
    else
    {
        port_inout_mode &= ~(1 << pin_num);
    }
    ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_OUTPUT_P0 : AW9523_REG_OUTPUT_P1, port_inout_mode);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set pin in/out mode failed");
        return;
    }
}

/**
 * @brief If D[4]=0, the P0 port is in Open-Drain mode (external pull-up resistor is required); If D[4]=1, P0 is in push-pull mode.
    D[1:0] port output maximum current value, I-max default is 37ma, 00: 37mA, 01: 27.75mA, 10: 18.5mA, 11: 9.25mA.
    D[7:5], D[3:2] are left by default and must be set to 0 if changes are needed
 * @param ctrl_cmd_byte
 */
void aw9523_set_ctl(uint8_t ctrl_cmd_byte)
{
    esp_err_t ret = aw9523_reg_write_byte(AW9523_REG_CTRL, ctrl_cmd_byte);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set ctl failed");
        return;
    }
}

/**
 * @brief Sets the LED maximum current
 * @param current
 */
void aw9523_set_led_max_current(AW9523_Current_t current)
{
    uint8_t current_temp;
    esp_err_t ret = aw9523_reg_read(AW9523_REG_CTRL, &current_temp, sizeof(current_temp));
    current_temp &= 0xFC; // 清除 D[1:0]
    current_temp |= current;

    ret = aw9523_reg_write_byte(AW9523_REG_CTRL, current_temp);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set led I-max failed");
        return;
    }
}

/**
 * @brief Settings Specify the GPIO or LED mode of the port
 * @param port
 * @param mode_byte D[7:0] set Px_7 ~ Px_0, 0: LED mode; 1: GPIO mode(default)
 */
void aw9523_set_port_gpio_or_led(AW9523_Port_t port, uint8_t mode_byte)
{
    esp_err_t ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_MODE_P0 : AW9523_REG_MODE_P1, mode_byte);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set port mode failed");
        return;
    }
}

/**
 * @brief Settings Specify the GPIO or LED mode of the port_pin
 * @param port
 * @param pin_num
 * @param mode_bit 0: LED mode; 1: GPIO mode(default)
 */
void aw9523_set_pin_gpio_or_led(AW9523_Port_t port, AW9523_Pin_t pin_num, AW9523_Mode_t mode_bit)
{
    uint8_t port_mode;
    esp_err_t ret = aw9523_reg_read((port == AW9523_PORT_0) ? AW9523_REG_MODE_P0 : AW9523_REG_MODE_P1, &port_mode, sizeof(port_mode));
    if (mode_bit == 1)
    {
        port_mode |= (1 << pin_num);
    }
    else
    {
        port_mode &= ~(1 << pin_num);
    }
    ret = aw9523_reg_write_byte((port == AW9523_PORT_0) ? AW9523_REG_MODE_P0 : AW9523_REG_MODE_P1, port_mode);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Set pin gpio/led mode failed");
        return;
    }
}

/**
 * @brief Set the LED duty of the specified port individually
 * @param port
 * @param duty_byte  0x00 ~ 0xff
 */
void aw9523_set_port_duty(AW9523_Port_t port, uint8_t duty_byte)
{
    uint8_t reg;
    esp_err_t ret;
    if (port == AW9523_PORT_0)
    {
        reg = 0x24;
        for (int i = 0; i < 8; i++)
        {
            ret = aw9523_reg_write_byte(reg + i, duty_byte);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Set port duty failed");
                return;
            }
        }
    }
    else
    {
        reg = 0x20;
        for (int i = 0; i < 4; i++)
        {
            ret = aw9523_reg_write_byte(reg + i, duty_byte);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Set port duty failed");
                return;
            }
        }
        reg = 0x2C;
        for (int i = 0; i < 4; i++)
        {
            ret = aw9523_reg_write_byte(reg + i, duty_byte);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Set port duty failed");
                return;
            }
        }
    }
}

/**
 * @brief Set the LED duty of the specified port_pin individually
 * @param port
 * @param pin_num
 * @param duty_byte  0x00 ~ 0xff
 */
void aw9523_set_pin_duty(AW9523_Port_t port, AW9523_Pin_t pin_num, uint8_t duty_byte)
{
    uint8_t reg;
    if (port == AW9523_PORT_0)
    {
        reg = 0x24 + pin_num;
    }
    else
    {
        reg = 0x20 + pin_num + ((pin_num > 3) ? 0x08 : 0x00);
    }
    aw9523_reg_write_byte(reg, duty_byte);
}

/**
 * @brief Read the AW9523 chip ID
 * @return Chip ID defaults to 0x23
 */
uint8_t aw9523_read_ID(void)
{
    uint8_t data = 0x00;
    esp_err_t ret = aw9523_reg_read(AW9523_REG_ID, &data, sizeof(data));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Read ID failed");
    }
    return data;
}

/**
 * @brief Reset AW9523
 */
void aw9523_reset(void)
{
    esp_err_t ret = aw9523_reg_write_byte(AW9523_REG_RST, 0x00);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "AW9523 Reset failed");
        return;
    }
}
/**
 * @brief AW9523 Demo
 */
void aw9523_Demo(void)
{
    aw9523_i2c_init();
    aw9523_reset();
    uint8_t id = aw9523_read_ID();
    ESP_LOGI(TAG, "AW9523 ID: 0x%02X", id);

    /* Set P1 to gpio mode  */
    aw9523_set_port_gpio_or_led(AW9523_PORT_1, 0xff);
    aw9523_set_port_inout(AW9523_PORT_1, AW9523_MODE_OUT);
    ESP_LOGI(TAG, "Port 1 level: 0x%02X", aw9523_read_level(AW9523_PORT_1));

    aw9523_set_port_level(AW9523_PORT_1, 0xaa);          // set port 1 pin p1_7~p1_0 = 1010 1010
    aw9523_set_pin_level(AW9523_PORT_1, AW9523_PX_0, 1); // set port 1 pin p1_7 = 1
    ESP_LOGI(TAG, "Port 1 level: 0x%02X", aw9523_read_level(AW9523_PORT_1));

    /* Set P0 to led mode */
    aw9523_set_port_gpio_or_led(AW9523_PORT_0, 0x00);
    aw9523_set_port_inout(AW9523_PORT_0, AW9523_MODE_OUT);
    aw9523_set_ctl(0x10);                          // Set the ctrl register to specify the output mode of P0 (open-drain/push-pull)
    aw9523_set_led_max_current(AW9523_CURR_18_5M); // Considering that most LEDs have a maximum limit current of 20 ma, the default setting is 18.5 ma

    aw9523_set_port_duty(AW9523_PORT_0, 0xff);             // set port 0 duty = 256/256 * 18.5mA = 18.5mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_1, 0x80); // set port 0 pin p0_1 duty = 128/256 * 18.5mA = 9.25mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_2, 0x40); // set port 0 pin p0_2 duty = 64/256 * 18.5mA = 4.625mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_3, 0x20); // set port 0 pin p0_3 duty = 32/256 * 18.5mA = 2.3125mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_4, 0x10); // set port 0 pin p0_4 duty = 16/256 * 18.5mA = 1.15625mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_5, 0x08); // set port 0 pin p0_5 duty = 8/256 * 18.5mA = 0.578125mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_6, 0x04); // set port 0 pin p0_6 duty = 4/256 * 18.5mA = 0.2890625mA
    aw9523_set_pin_duty(AW9523_PORT_0, AW9523_PX_7, 0x02); // set port 0 pin p0_7 duty = 2/256 * 18.5mA = 0.14453125mA
}