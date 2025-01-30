# ESP-IDF 示例 驱动拓展IO芯片 艾为AW9523

|支持的目标 |ESP32 系列 |ESP32-C3 系列 |ESP32-C6 系列 |ESP32-H2 系列 |ESP32-P4 开发板 |ESP32-S2 系列 |ESP32-S3 系列 |
|----------------- |----- |-------- |-------- |-------- |-------- |-------- |-------- |

## 如何使用示例

### 所需硬件

要运行此示例，您应该有一个基于 ESP32、ESP32-S、ESP32-C 或 ESP32-H 的开发板以及一个 AW9523。

#### 引脚分配：

```C
#define AW9523_I2C_ADDR 0x5b  /* I2C default address, which can be modified by hardware, at this time A0 = 1, A1 = 1 */
#define AW9523_SCL_IO GPIO_NUM_18          /* GPIO number used for I2C master clock */
#define AW9523_SDA_IO GPIO_NUM_17          /* GPIO number used for I2C master data  */
```

### 构建和 Flash
本例程测试基于 ESP-IDF v5.2.3
输入 `idf.py -p PORT flash monitor` 来构建、烧录和监控项目。

（要退出串行监视器，请键入 ``Ctrl-]``。 请参阅 [入门指南]（https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html） 了解配置和使用 ESP-IDF 构建项目的完整步骤。

## 例程输出

```bash
I (324) AW9523: AW9523 ID: 0x23
I (324) AW9523: Port 1 level: 0xFF
I (334) AW9523: Port 1 level: 0xAB
```
此时, 
- P1_7 level : VCC (3.3V / 5V)
- P1_6 level : GND
- P1_5 level : VCC
- P1_4 level : GND
- P1_3 level : VCC
- P1_2 level : GND
- P1_1 level : VCC
- P1_0 level : VCC

请将测试 LED 的负极连接到 P0_7~P0_0，正极连接到 VCC
此时
- p0_7 : 18.5 mA
- p0_6 : 9.25 mA
- p0_5 : 4.625 mA
- p0_4 : 2.3125 mA
- p0_3 : 1.15625 mA
- p0_2 : 0.578125 mA
- p0_1 : 0.2890625 mA
- p0_0 : 0.14453125 mA