#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "bldc_as5600.h"

#include "pid.h"
#include "fast_math.h"
#include "filter.h"
#include "foc_config.h"
#include "foc.h"

#define TAG "bldc_module"
#define BLDC_USE_MCPWM

#define AS5600_DEFAULT_ADDRESS 0x36
/* Raw angle registers */
#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D
#define AS_2PI 6.28318530718f
#define AS_FULL_CR 360.0f
uint8_t m_errCnt = 0;
bool m_taskFlag = false;
esp_timer_handle_t angleSetTimerHd = nullptr;

void foc_start_task(void);
void foc_stop_task(void);

As5600Sensor* m_as5600Sensor_ = nullptr;
void set_as5600hd(As5600Sensor* As5600Sensor_) {
    m_as5600Sensor_ = As5600Sensor_;
}

As5600Sensor* get_as5600hd(void) {
    return m_as5600Sensor_;
}

As5600Sensor::As5600Sensor(i2c_master_bus_handle_t i2c_bus) 
    : I2cDevice(i2c_bus, AS5600_DEFAULT_ADDRESS) {
}

uint16_t As5600Sensor::AS5600GetRawAngle(void)
{
    uint8_t buf[2];
    // ReadRegs(AS5600_RAW_ANGLE_L, &buf[0], 1);
    // ReadRegs(AS5600_RAW_ANGLE_H, &buf[1], 1);
    buf[0] = ReadReg(AS5600_RAW_ANGLE_L);
    buf[1] = ReadReg(AS5600_RAW_ANGLE_H);

    if ((buf[0] == 0xff) && (buf[1] == 0xff)) {
        m_errCnt++;
        if (m_errCnt == 5) {
            //foc_stop_task();
            esp_timer_stop(angleSetTimerHd);
        }
        return 0;
    }
    
    uint16_t data = (buf[0] + (buf[1] << 8));

    if (m_taskFlag == false) {
        m_errCnt = 0;
        //foc_start_task();
    }
    return data;
}

float As5600Sensor::AS5600GetSensorAngle(void) {
  // (number of full rotations)*2PI + current sensor angle 
  return  ( AS5600GetRawAngle()  * AS_FULL_CR / (float)4096);
}

float As5600Sensor::AS5600GetSensorRad(void) {
  // (number of full rotations)*2PI + current sensor angle 
  return  ( AS5600GetRawAngle()  * AS_2PI / (float)4096);
}

As5600Sensor* m_as5600 = nullptr;
Pca9685* m_pca9685 = nullptr;
esp_timer_handle_t focLoopTimerHd = nullptr;
esp_timer_handle_t controllerLoopTimerHd = nullptr;
esp_timer_handle_t speed_updateLoopTimerHd = nullptr;
esp_timer_handle_t focTestLoopTimerHd = nullptr;

static float m_bldcTargetAngle = 0;
static float m_bldcCurAngle = 0;
static float m_bldcAdjTAngle = 0;
static bool m_adjDir = true;
#define BLDC_IN1_CHANNEL (8)
#define BLDC_IN2_CHANNEL (9)
#define BLDC_IN3_CHANNEL (10)

void foc_start_task(void) {
    esp_timer_start_periodic(focLoopTimerHd, 10000); // 0.1ms
    esp_timer_start_periodic(controllerLoopTimerHd, 5 * 100000); // 5ms
    esp_timer_start_periodic(speed_updateLoopTimerHd, 250000); // 2.5ms
    m_taskFlag = true;
}

void foc_stop_task(void) {
    esp_timer_stop(focLoopTimerHd);
    esp_timer_stop(controllerLoopTimerHd);
    esp_timer_stop(speed_updateLoopTimerHd);
    m_taskFlag = false;
}

void foc_test_loop(void) {
    if (m_as5600Sensor_ == nullptr) {
        return;
    }
    static uint8_t as5600_debug = 0;
    int anglet = 0;

    // 3s 读一次角度值做检查
    as5600_debug++;
    if ((as5600_debug % 3) == 0) {
        float ang1 = m_as5600Sensor_->AS5600GetSensorAngle();
        ESP_LOGI(TAG, "\r\n AS5600 ANG:%0.2f°", ang1);
    }
    if (as5600_debug == 13) {
        float ang = m_as5600Sensor_->AS5600GetSensorAngle();
        ESP_LOGI(TAG, "\r\n AS5600 ANG:%0.2f°", ang);
        anglet = ang + 75;
        BLDCSetAngle(anglet);
        as5600_debug = 0;
    }
}

void angle_set_timercb(void) {
    float tmpAngle = m_as5600Sensor_->AS5600GetSensorRad();
    if (m_adjDir == true) {
        if (tmpAngle < m_bldcTargetAngle) {
            m_bldcAdjTAngle += 0.1;
        } else {
            ESP_LOGI(TAG, "\r\n angle_set_timercb done");
            //foc_set_phase_dutycycle(0, 0, 0);
            esp_timer_stop(angleSetTimerHd);
            return;
        }
    } else {
        if (tmpAngle > m_bldcTargetAngle) {
            m_bldcAdjTAngle -= 0.1;
        } else {
            ESP_LOGI(TAG, "\r\n angle_set_timercb done");
            //foc_set_phase_dutycycle(0, 0, 0);
            esp_timer_stop(angleSetTimerHd);
            return;
        }
    }
    foc_set_phase_dutycycle(m_bldcAdjTAngle, 0.5, 0);
}

void foc_init(void) {
    BLDC_MotorPwmInit();
    /* initialize the low pass filter */
    init_lpf(&velocity_filter, TWO_PI/SPEED_UP_FREQ);

    /* initialize the PID controller */
    //pid_config(SPEED_LOOP_CONTROL);
    pid_config(ANGLE_LOOP_CONTROL);
    /* initialize the FOC phase */
    foc_set_phase_dutycycle(0.1, 0.5, 0);
    // foc_calibrate_phase();

    /* correct the mechanical angle zero deviation */
    encoder_zeroing();

    /* create Task for foc_loop, controller_loop and speed update loop */
    esp_timer_create_args_t foc_loop_timer_args = {
        .callback = [](void* arg) {
            foc_loop();
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "foc_loop",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&foc_loop_timer_args, &focLoopTimerHd);

    esp_timer_create_args_t controller_loop_timer_args = {
        .callback = [](void* arg) {
            controller_loop();
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "controller_loop",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&controller_loop_timer_args, &controllerLoopTimerHd);

    esp_timer_create_args_t speed_update_loop_timer_args = {
        .callback = [](void* arg) {
            encoder_update_speed();
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "speed_update_loop",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&speed_update_loop_timer_args, &speed_updateLoopTimerHd);

    esp_timer_create_args_t angle_set_timer_args = {
        .callback = [](void* arg) {
            angle_set_timercb();
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "angle_set_loop",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&angle_set_timer_args, &angleSetTimerHd);

    esp_timer_create_args_t foc_test_loop_timer_args = {
        .callback = [](void* arg) {
            foc_test_loop();
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "foc_test_loop",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&foc_test_loop_timer_args, &focTestLoopTimerHd);
    esp_timer_start_periodic(focTestLoopTimerHd, 1000 * 1000); // 1s

    //foc_start_task();
}

void BLDCModuleInit(As5600Sensor* as5600, Pca9685* pca9685) {
    m_as5600 = as5600;
    m_pca9685 = pca9685;
    foc_init();

    //speed test
    //pid_control_mode_flag = SPEED_LOOP_CONTROL;
    //speed_pid_handler.expect = _PI;
    pid_control_mode_flag = ANGLE_LOOP_CONTROL;
}

void BLDCSetAngle(float angle) {
    float anglerad = angle * AS_2PI / AS_FULL_CR;
    // angle_pid_handler.expect = anglerad;
    // angle_pid_handler.expect = _normalizeAngle(angle_pid_handler.expect);
    if (anglerad > AS_2PI) {
        anglerad = anglerad - AS_2PI;
    }
    m_bldcCurAngle = m_as5600Sensor_->AS5600GetSensorRad();
    if (anglerad > m_bldcCurAngle) {
        m_adjDir = true;
    } else {
        m_adjDir = false;
    }
    m_bldcAdjTAngle = m_bldcCurAngle;

    m_bldcTargetAngle = anglerad;

    ESP_LOGI(TAG, "\r\n BLDCSetAngle ANG:%0.2f°", m_bldcTargetAngle);
    esp_timer_start_periodic(angleSetTimerHd, 5 * 1000); // 10ms
    // foc_set_phase_dutycycle(angle_pid_handler.expect, 1.5, 0);
}

float BLDC_foc_get_angle(void) {
    return m_as5600->AS5600GetSensorRad();
}

void BLDC_foc_set_pwm_duty(float ch0, float ch1, float ch2) {
    m_pca9685->SetPwm(BLDC_IN1_CHANNEL, (uint16_t)ch0);
    m_pca9685->SetPwm(BLDC_IN2_CHANNEL, (uint16_t)ch1);
    m_pca9685->SetPwm(BLDC_IN3_CHANNEL, (uint16_t)ch2);
}

#define MO1_PIN 11
#define MO2_PIN 10
#define MO3_PIN 20

#define MOTOR_MCPWM_UNIT MCPWM_UNIT_0

void BLDC_SetMotorPwmDuty(float ch0, float ch1, float ch2) {
    // 更新 PWM 占空比
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, ch0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, ch1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ch2);
    
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);

}

void BLDC_SetPwmDuty(float ch0, float ch1, float ch2) {
#ifdef BLDC_USE_MCPWM
    return BLDC_SetMotorPwmDuty(ch0, ch1, ch2);
#else
    return BLDC_foc_set_pwm_duty(ch0, ch1, ch2);
#endif
}

void BLDC_MotorPwmInit(void) {
    // 初始化 MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,MO1_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM1A,MO2_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM2A,MO3_PIN);

    mcpwm_config_t pwm_config = {
        .frequency = 20000,  // 设置 PWM 频率为 20kHz
        .cmpr_a = 0,  // 初始占空比为 0
        .cmpr_b = 0,      
        .duty_mode = MCPWM_DUTY_MODE_0,  // 设置占空比计算方式为 MCPWM_DUTY_MODE_0
        .counter_mode = MCPWM_UP_COUNTER,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    ESP_LOGI(TAG, "motor pwm init success");
    ESP_LOGI(TAG, "MCPWM0A,MO1_PIN:%d",MO1_PIN);
    ESP_LOGI(TAG, "MCPWM1A,MO2_PIN:%d",MO2_PIN);
    ESP_LOGI(TAG, "MCPWM2A,MO3_PIN:%d",MO3_PIN);
}