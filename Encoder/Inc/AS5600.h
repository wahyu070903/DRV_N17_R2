/*
 * AS5600.h
 *
 *  Created on: Mar 1, 2025
 *      Author: Orsted
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define AS5600_SLAVE_ADDRESS 0x36
/* AS5600 Configuration Registers */
#define AS5600_REGISTER_ZMCO 0x00
#define AS5600_REGISTER_ZPOS_HIGH 0x01
#define AS5600_REGISTER_ZPOS_LOW 0x02
#define AS5600_REGISTER_MPOS_HIGH 0x03
#define AS5600_REGISTER_MPOS_LOW 0x04
#define AS5600_REGISTER_MANG_HIGH 0x05
#define AS5600_REGISTER_MANG_LOW 0x06
#define AS5600_REGISTER_CONF_HIGH 0x07
#define AS5600_REGISTER_CONF_LOW 0x08
/* AS5600 Output Registers */
#define AS5600_REGISTER_RAW_ANGLE_HIGH 0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW 0x0D
#define AS5600_REGISTER_ANGLE_HIGH 0x0E
#define AS5600_REGISTER_ANGLE_LOW 0x0F
/* AS5600 Status Registers */
#define AS5600_REGISTER_STATUS 0x0B
#define AS5600_REGISTER_AGC 0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH 0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW 0x1C
#define AS5600_REGISTER_BURN 0xFF

/* AS5600 Configuration Settings */
#define AS5600_POWER_MODE_NOM 1
#define AS5600_POWER_MODE_LPM1 2
#define AS5600_POWER_MODE_LPM2 3
#define AS5600_POWER_MODE_LPM3 4
#define AS5600_POWER_MODE_DEFAULT AS5600_POWER_MODE_NOM
#define AS5600_HYSTERESIS_OFF 1
#define AS5600_HYSTERESIS_1LSB 2
#define AS5600_HYSTERESIS_2LSB 3
#define AS5600_HYSTERESIS_3LSB 4
#define AS5600_HYSTERESIS_DEFAULT AS5600_HYSTERESIS_OFF
#define AS5600_OUTPUT_STAGE_FULL 1 /* Ratiometric analog output ranging from GND-VCC*/
#define AS5600_OUTPUT_STAGE_REDUCED 2 /* Ratiometric analog output ranging from 10% to 90% of VCC */
#define AS5600_OUTPUT_STAGE_PWM 3 /* Digital PWM output */
#define AS5600_OUTPUT_STAGE_DEFAULT AS5600_OUTPUT_STAGE_FULL
#define AS5600_PWM_FREQUENCY_115HZ 1
#define AS5600_PWM_FREQUENCY_230HZ 2
#define AS5600_PWM_FREQUENCY_460HZ 3
#define AS5600_PWM_FREQUENCY_920HZ 4
#define AS5600_PWM_FREQUENCY_DEFAULT AS5600_PWM_FREQUENCY_115HZ
#define AS5600_SLOW_FILTER_16X 1
#define AS5600_SLOW_FILTER_8X 2
#define AS5600_SLOW_FILTER_4X 3
#define AS5600_SLOW_FILTER_2X 4
#define AS5600_SLOW_FILTER_DEFAULT AS5600_SLOW_FILTER_16X
#define AS5600_FAST_FILTER_SLOW_ONLY 1
#define AS5600_FAST_FILTER_6LSB 2
#define AS5600_FAST_FILTER_7LSB 3
#define AS5600_FAST_FILTER_9LSB 4
#define AS5600_FAST_FILTER_18LSB 5
#define AS5600_FAST_FILTER_21LSB 6
#define AS5600_FAST_FILTER_24LSB 7
#define AS5600_FAST_FILTER_10LSB 8
#define AS5600_FAST_FILTER_DEFAULT AS5600_FAST_FILTER_SLOW_ONLY
#define AS5600_WATCHDOG_OFF 1
#define AS5600_WATCHDOG_ON 2
#define AS5600_WATCHDOG_DEFAULT AS5600_WATCHDOG_ON

/* AS5600 Status Definitions */
#define AS5600_AGC_MIN_GAIN_OVERFLOW (uint8_t)(1UL << 3) /*Error bit indicates b-field is too string */
#define AS5600_AGC_MAX_GAIN_OVERFLOW (uint8_t)(1UL << 4) /*Error bit indicates b-field is too weak */
#define AS5600_MAGNET_DETECTED (uint8_t)(1UL << 5) /*Status bit indicates b-field is detected */

#define AS5600_DIR_CW 1
#define AS5600_DIR_CCW 2

#define AS5600_12_BIT_MASK (uint16_t)4095
typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    uint8_t i2cAddr;
    GPIO_TypeDef *DirPort;
    uint16_t DirPin;
    uint8_t PositiveRotationDirection;
    uint8_t LowPowerMode;
    uint8_t Hysteresis;
    uint8_t OutputMode;
    uint8_t PWMFrequency;
    uint8_t SlowFilter;
    uint8_t FastFilterThreshold;
    uint8_t WatchdogTimer;

    /* Private */
    volatile uint8_t confRegister[2];

} AS5600_TypeDef;

typedef enum {
	AS5600_INIT_OK = 0x00,
	AS5600_INIT_ERR = 0x01,
	AS5600_INIT_MAG_NOT = 0x02,
	AS5600_INIT_MAG_STG = 0x03,
	AS5600_INIT_MAG_WEK = 0x04,
	AS5600_INIT_HAL_FAIL = 0x05,
} AS5600Init_StatusTypedef;
/***********************    FUNCTION PROTOTYPES    ***********************/

AS5600_TypeDef *AS5600_New(void);
AS5600Init_StatusTypedef AS5600_Init(AS5600_TypeDef *a);

HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef *const a,
                                          const uint16_t pos);
HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef *const a,
                                         const uint16_t pos);
HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef *const a,
                                     const uint16_t angle);

HAL_StatusTypeDef AS5600_SetPositiveRotationDirection(AS5600_TypeDef *const a, const uint8_t dir);

HAL_StatusTypeDef AS5600_SetLowPowerMode(AS5600_TypeDef *const a,
                                         const uint8_t mode);
HAL_StatusTypeDef AS5600_SetHysteresis(AS5600_TypeDef *const a,
                                       const uint8_t hysteresis);
HAL_StatusTypeDef AS5600_SetOutputMode(AS5600_TypeDef *const a,
                                       const uint8_t mode, uint8_t freq);
HAL_StatusTypeDef AS5600_SetSlowFilter(AS5600_TypeDef *const a,
                                       const uint8_t mode);
HAL_StatusTypeDef AS5600_SetFastFilterThreshold(AS5600_TypeDef *const a,
                                                const uint8_t threshold);
HAL_StatusTypeDef AS5600_SetWatchdogTimer(AS5600_TypeDef *const a, const uint8_t mode);

HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef *const a,
                                     uint16_t *const angle);
HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef *const a,
                                  uint16_t *const angle);
HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef *const a,
                                         uint8_t *const stat);
HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef *const a,
                                       uint8_t *const agc);
HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef *const a, uint16_t *const mag);

#endif /* INC_AS5600_H_ */
