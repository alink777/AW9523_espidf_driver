# Esp-idf driver for AW9523B 16 multi-function LED driver and GPIO controller

| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

## How to use example

### Hardware Required

To run this example, you should have one ESP32, ESP32-S, ESP32-C or ESP32-H based development board as well as a AW9523. 

#### Pin Assignment:

```C
#define AW9523_I2C_ADDR 0x5b  /* I2C default address, which can be modified by hardware, at this time A0 = 1, A1 = 1 */
#define AW9523_SCL_IO GPIO_NUM_18          /* GPIO number used for I2C master clock */
#define AW9523_SDA_IO GPIO_NUM_17          /* GPIO number used for I2C master data  */
```

### Build and Flash

This example test is based on ESP-IDF v5.2.3
Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash
I (324) AW9523: AW9523 ID: 0x23
I (324) AW9523: Port 1 level: 0xFF
I (334) AW9523: Port 1 level: 0xAB
```
at this time, 
- P1_7 level : VCC (3.3V / 5V)
- P1_6 level : GND
- P1_5 level : VCC
- P1_4 level : GND
- P1_3 level : VCC
- P1_2 level : GND
- P1_1 level : VCC
- P1_0 level : VCC

Please connect the negative pole of the LED to P0 7~P0 0, and the positive pole to VCC
- p0_7 : 18.5 mA
- p0_6 : 9.25 mA
- p0_5 : 4.625 mA
- p0_4 : 2.3125 mA
- p0_3 : 1.15625 mA
- p0_2 : 0.578125 mA
- p0_1 : 0.2890625 mA
- p0_0 : 0.14453125 mA

