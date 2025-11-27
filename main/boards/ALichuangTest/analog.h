#ifndef _ANALOG_MAGIC_H_
#define _ANALOG_MAGIC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_timer.h>
#include "skills/motion.h"
#include "bldc_as5600.h"

/* 采样通道定义 */
typedef enum {
    BATTERY_SAMPLE = 0,
    BODY_TYPE_SAMPLE,
    ANGLE_SENSOR_SAMPLE,
    ADC_TEST_SAMPLE,
    ALL_SAMPLE_NUMS
} sample_channel;


/* 电池采样功能公共定义区 */
#define MIN_BAT_VOL (3520) // 测量值 3.52V
#define MAX_BAT_VOL (4200) // 测量值 4.2V

/* 身体类型采样功能公共定义区 */
/* 依据实体类型可以更改命名，原函数一起改 */
typedef enum {
    BODY_TYPE_1 = 0,
    BODY_TYPE_2,
    BODY_TYPE_3,
    BODY_TYPE_4,
    BODY_TYPE_5,
    BODY_TYPE_6,
    BODY_TYPE_7,
    BODY_TYPE_DEFAULT,
    BODY_TYPE_9,
    BODY_TYPE_10,
    BODY_TYPE_11,
    BODY_TYPE_12,
    BODY_TYPE_13,
    BODY_TYPE_14,
    BODY_TYPE_15,
    BODY_TYPE_16,
    BODY_TYPE_17,
    BODY_TYPE_18,
    ALL_BODY_TYPE_NUMS,
    INVALID_TYPE = 255
} body_type_def;

typedef struct {
    body_type_def type;
    int lowerVol; // mV
    int upperVol; // mV
    char infoP[32];
} body_type_info;

/* 角度传感器采样功能公共定义区 */
#define BODY_CENTER_ANGLE (3250)
#define TOLERANCE_ANGLE   (1600) // 暂时按照全局可转设计


// ADC初始化
void DRV_AdcInit(void);
// 获取通道采样值，转化为mV
int DRV_GetSampleData(sample_channel channel);
// 获取当前电池电压，转化为mV
int DRV_GetBatVol(void);
// 获取当前板类型号
body_type_def DRV_GetBodyType(void);
// 获取当前角度值
int GetAngle(void);

/* 采样通道定义 */
typedef enum {
    CW_LEFT = 0,
    CCW_RIGHT
} rotate_dir;

class AngleSensor {
public:
    AngleSensor(Motion* motion);
    ~AngleSensor();

    void SetRotate(rotate_dir dir, int angle);
    void BackToCenter(void);

private:
    Motion* m_motion; // 空心杯电机控制句柄
    int m_centerPosAng;
    int m_toleranceAng;
    int m_angleNow;
    int m_smallCircle = 10;
    int m_bigCircle = 64;
    // 记录已经下发给马达任务的“期望马达角度”（单位：度，马达轴角度）
    // 仅用于通过队列(异步任务)发相对转动时累加为绝对角度目标
    float m_expectedMotorAngle = 0.0f;

    void FreshAngle(void);
};

void set_anglehd(AngleSensor* AngleSensor_);

#endif /* _ANALOG_MAGIC_H_ */
