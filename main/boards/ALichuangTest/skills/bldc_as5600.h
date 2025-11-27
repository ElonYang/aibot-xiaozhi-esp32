#ifndef _BLDC_AS5600_SENSOR_H
#define _BLDC_AS5600_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "i2c_device.h"
#include <esp_err.h>
#include "pca9685.h"

class As5600Sensor : public I2cDevice {
public:
    As5600Sensor(i2c_master_bus_handle_t i2c_bus);

    // 获取原始采样值
    uint16_t AS5600GetRawAngle(void);

    // 获取换算后角度值
    float AS5600GetSensorAngle(void);

    // 获取换算后弧度值，foc用
    float AS5600GetSensorRad(void);
};

void set_as5600hd(As5600Sensor* As5600Sensor_);
As5600Sensor* get_as5600hd(void);
// 无刷电机初始化，传入磁编码器与pwm控制模块实例
void BLDCModuleInit(As5600Sensor* as5600, Pca9685* pca9685);

// 传入电机目标角度 0-360.0
void BLDCSetAngle(float angle);

// for foc
float BLDC_foc_get_angle(void);
void BLDC_SetPwmDuty(float ch0, float ch1, float ch2);
void BLDC_MotorPwmInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLDC_AS5600_SENSOR_H */