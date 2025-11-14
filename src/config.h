#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

// LED STRIP 最大亮度
#ifndef CONFIG_LED_STRIP_BRIGHTNESS
    #define CONFIG_LED_STRIP_BRIGHTNESS (0x80)
#endif

// ====================================================================================
// 硬件版本对照表
// ------------------------------------------------------------------------------------
// | 引脚/参数   | V2 (开源硬件 V2.6.0) | V3 (开源硬件 V3.1.2) | V255 (测试版本)      |
// | RGB_NUM     | 1                    | 4                    | 4                    |
// | VBUS_ADC    | PA1 (ADC1)           | PB0 (ADC8)           | PB0 (ADC8)           |
// | VBUS_DIV_R1 | 68kΩ                 | 137kΩ                | 100kΩ                |
// | VBUS_DIV_R2 | 4.7kΩ                | 10kΩ                 | 10kΩ                 |
// | CC_EN       | PB0                  | PA4                  | PA4                  |
// | CC_RD_EN    | PB12                 | —                    | —                    |
// | CC1_RD_EN   | —                    | PB1                  | PB1                  |
// | CC2_RD_EN   | —                    | PB12                 | PB12                 |
// | CC1_RA_EN   | —                    | PB3                  | PB3                  |
// | CC2_RA_EN   | —                    | PB11                 | PB11                 |
// ====================================================================================
#ifndef CONFIG_HW_VERSION
    #define CONFIG_HW_VERSION (2)
#endif

#endif  // DEVICE_CONFIG_H