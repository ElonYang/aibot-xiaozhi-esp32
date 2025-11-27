#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"
#include "skills/vibration.h"
#include "skills/motion.h"
#include "qmi8658.h"
#include "interaction/core/event_engine.h"
#include "interaction/upload/event_uploader.h"
#include "interaction/controller/mcp_response_controller.h"
#include "interaction/controller/local_response_controller.h"
#include "pca9685.h"
#include "i2c_bus_manager.h"
#include "analog.h"
#include "device_state_event.h"
#include "skills/bldc_as5600.h"

#if CONFIG_ENABLE_BLUETOOTH_PROVISIONING
#include "boards/ALichuangTest/bluetooth_provisioning/blufi_provisioning.h"
#endif

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_timer.h>
#include <mutex>

// 人脸模型
#include "skills/human_face_detect.hpp"
#include "dl_image_jpeg.hpp"

/* SD Card */
#include "sddata_pro.h"
/* SD Card End */

#if CONFIG_LINGXI_ANIMA_UI
#include "skills/animation.h"
#elif CONFIG_XIAOZHI_DEFAULT_UI
#include "display/lcd_display.h"
#endif

#define TAG "ALichuangTest"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xe0); // 0-4输出 5-6默认输入
        //WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
private:
    Pca9557* pca9557_;

public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557) 
        : BoxAudioCodec(i2c_bus, 
                       AUDIO_INPUT_SAMPLE_RATE, 
                       AUDIO_OUTPUT_SAMPLE_RATE,
                       AUDIO_I2S_GPIO_MCLK, 
                       AUDIO_I2S_GPIO_BCLK, 
                       AUDIO_I2S_GPIO_WS, 
                       AUDIO_I2S_GPIO_DOUT, 
                       AUDIO_I2S_GPIO_DIN,
                       GPIO_NUM_NC, 
                       AUDIO_CODEC_ES8311_ADDR, 
                       AUDIO_CODEC_ES7210_ADDR, 
                       AUDIO_INPUT_REFERENCE),
          pca9557_(pca9557) {
    }

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};

class ALichuangTest : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    Pca9557* pca9557_;
    Esp32Camera* camera_;
    Qmi8658* imu_ = nullptr;
    EventEngine* event_engine_ = nullptr;
    EventUploader* event_uploader_ = nullptr;
    esp_timer_handle_t event_timer_ = nullptr;
    Pca9685* pca9685_ = nullptr;           // PCA9685 PWM控制器
    Vibration* vibration_skill_ = nullptr; // 振动技能管理器
    Motion* motion_skill_ = nullptr;       // 直流马达动作控制技能
    AngleSensor* angle_sensor = nullptr;
    McpResponseController* mcp_response_controller_ = nullptr; // MCP响应控制器
    LocalResponseController* local_response_controller_ = nullptr; // 本地响应控制器
    TaskHandle_t delay_task_handle = nullptr;
    SDdata_Pro* sdhccard = nullptr;
    As5600Sensor* as5600 = nullptr;

#if CONFIG_LINGXI_ANIMA_UI
    // 动画相关成员变量
    std::string current_animation_ = "idle_q1";  // 默认使用idle_q1，启动后会根据情感状态调整
    mutable std::mutex animation_mutex_;
    AnimaDisplay* display_;
    TaskHandle_t animation_task_handle_ = nullptr; // 动画播放任务句柄

    void StartAnimationPlay() {
        // 设置动画变化回调
        auto display = GetDisplay();
        if (display) {
            display->OnAnimationChanged([this](const std::string& animation) {
                ESP_LOGI(TAG, "接收到动画变化回调: %s", animation.c_str());

                // 将通用情绪指令映射为具体动画，避免出现未知动画日志
                if (animation == "neutral") {
                    auto& app = Application::GetInstance();
                    auto state = app.GetDeviceState();

                    // 仅在待机状态下进行映射，监听状态交由本地响应系统处理
                    if (state == kDeviceStateIdle) {
                        auto& emotion_engine = EmotionEngine::GetInstance();
                        EmotionQuadrant quadrant = emotion_engine.GetQuadrant();

                        std::string mapped_animation;
                        // 待机状态：根据象限选择 idle_q1~q4
                        mapped_animation = AnimaDisplay::GetIdleAnimationByQuadrant(quadrant);
                        ESP_LOGI(TAG, "映射 neutral -> %s (idle)", mapped_animation.c_str());

                        SetCurrentAnimation(mapped_animation);
                        return; // 已处理
                    }

                    // 非 idle 状态忽略 neutral，保持当前动画，由各自流程处理（例如 listening 由 LocalResponse 控制）
                    ESP_LOGI(TAG, "忽略 neutral 于当前状态: %d", static_cast<int>(state));
                    return;
                }

                // 默认行为：按收到的名称设置
                SetCurrentAnimation(animation);
            });
        }
        xTaskCreate(AnimationPlayTask, "anim_play", 6144, this, 3, &animation_task_handle_);
        ESP_LOGI(TAG, "动画播放任务已启动");
    }
    // 获取当前动画状态
    std::string GetCurrentAnimation() {
        std::lock_guard<std::mutex> lock(animation_mutex_);
        return current_animation_;
    }

    // 设置当前动画状态
    void SetCurrentAnimation(const std::string& animation) {
        std::lock_guard<std::mutex> lock(animation_mutex_);
        current_animation_ = animation;
        ESP_LOGI(TAG, "动画状态变更为: %s", animation.c_str());
    }
    
    static void AnimationPlayTask(void* arg) {
        ALichuangTest* board = static_cast<ALichuangTest*>(arg);
        AnimaDisplay* display = board->GetDisplay();

        if (!display) {
            ESP_LOGE(TAG, "无法获取显示设备");
            vTaskDelete(NULL);
            return;
        }

        // 获取AudioProcessor实例的事件组 - 从application.h中直接获取
        auto& app = Application::GetInstance();

        // ====== 等待开机动画播放完成 ======
        // 等待开机动画播放完成，避免与idle动画冲突
        ESP_LOGI(TAG, "动画任务启动，等待开机动画完成...");
        while (!app.IsBootAnimationCompleted()) {
            vTaskDelay(pdMS_TO_TICKS(500));  // 每500ms检查一次
        }
        ESP_LOGI(TAG, "开机动画完成，开始加载idle动画");
        // =====================================

        // 根据当前情感象限选择初始idle动画
        auto& emotion_engine = EmotionEngine::GetInstance();
        EmotionQuadrant quadrant = emotion_engine.GetQuadrant();
        std::string initial_idle = AnimaDisplay::GetIdleAnimationByQuadrant(quadrant);

        ESP_LOGI(TAG, "初始化动画: %s (V=%.2f, A=%.2f, 象限=%d)",
                initial_idle.c_str(),
                emotion_engine.GetValence(),
                emotion_engine.GetArousal(),
                static_cast<int>(quadrant));

        // 设置当前动画为选择的idle动画
        board->SetCurrentAnimation(initial_idle);

        // 播放idle动画（无限循环）
        display->SetAnima(initial_idle, -1);
        ESP_LOGI(TAG, "初始idle动画已加载（无限循环）");

        // 持续监控和处理GIF动画播放
        // 定义用于判断是否正在播放音频的变量
        bool isAudioPlaying = false;

        // 定义用于检测动画变化的变量
        std::string lastAnimation = board->GetCurrentAnimation();

        // 定义用于判断是否应该播放情感动画的变量
        bool shouldPlayAnimation = false;

        // 定义自动回归neutral的超时机制（10秒无音频播放后自动回到neutral）
        TickType_t lastAudioTime = xTaskGetTickCount();
        const TickType_t neutralTimeout = pdMS_TO_TICKS(10000); // 10秒超时

        while (true) {
            // 检查动画是否发生变化
            std::string currentAnimation = board->GetCurrentAnimation();
            if (currentAnimation != lastAnimation) {
                ESP_LOGI(TAG, "动画变化检测: %s -> %s", lastAnimation.c_str(), currentAnimation.c_str());

                // 判断是否是idle动画，如果是则无限循环
                bool isNewIdleAnimation = (currentAnimation == "idle_q1" || currentAnimation == "idle_q2" ||
                                          currentAnimation == "idle_q3" || currentAnimation == "idle_q4");

                if (isNewIdleAnimation) {
                    display->SetAnima(currentAnimation, -1);  // 无限循环
                    ESP_LOGI(TAG, "已切换到idle动画: %s（无限循环）", currentAnimation.c_str());
                } else {
                    display->SetAnima(currentAnimation);  // 使用默认播放次数
                    ESP_LOGI(TAG, "已切换到新GIF动画: %s", currentAnimation.c_str());
                }
                lastAnimation = currentAnimation;
            }

            // 检查是否正在播放音频 - 使用应用程序状态判断
            isAudioPlaying = (app.GetDeviceState() == kDeviceStateSpeaking);

            // 更新最后一次音频播放时间
            if (isAudioPlaying) {
                lastAudioTime = xTaskGetTickCount();
            }

            // 检查是否需要自动回归idle状态
            TickType_t timeSinceLastAudio = xTaskGetTickCount() - lastAudioTime;
            // 判断是否是idle动画（idle_q1~q4）
            bool isIdleAnimation = (currentAnimation == "idle_q1" || currentAnimation == "idle_q2" ||
                                   currentAnimation == "idle_q3" || currentAnimation == "idle_q4");

            if (!isAudioPlaying && !isIdleAnimation && timeSinceLastAudio > neutralTimeout) {
                // 根据当前情感象限获取对应的idle动画
                auto& emotion_engine = EmotionEngine::GetInstance();
                EmotionQuadrant quadrant = emotion_engine.GetQuadrant();
                std::string idle_animation = AnimaDisplay::GetIdleAnimationByQuadrant(quadrant);

                ESP_LOGI(TAG, "长时间无音频播放, 自动回归idle状态: %s (V=%.2f, A=%.2f)",
                        idle_animation.c_str(),
                        emotion_engine.GetValence(),
                        emotion_engine.GetArousal());
                board->SetCurrentAnimation(idle_animation);
                // 注意：这里不直接修改currentAnimation, 让下次循环检测动画变化时处理
            }

            // 判断是否应该播放情感动画：非idle状态且正在说话
            shouldPlayAnimation = !isIdleAnimation && isAudioPlaying;

            // 输出调试信息（每10次循环输出一次，避免日志过多）
            static int debugCount = 0;
            if (++debugCount >= 10) {
                ESP_LOGD(TAG, "状态检查 - 动画: %s, 说话: %s, 播放动画: %s",
                    currentAnimation.c_str(),
                    isAudioPlaying ? "是" : "否",
                    shouldPlayAnimation ? "是" : "否");
                debugCount = 0;
            }

            // 短暂延时，避免CPU占用过高
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        // 释放资源（实际上不会执行到这里，除非任务被外部终止）
        vTaskDelete(NULL);
    }
#elif CONFIG_XIAOZHI_DEFAULT_UI
    LcdDisplay* display_;
#endif

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // 设置I2C总线管理器
        I2cBusManager::GetInstance()->SetBusHandle(i2c_bus_);

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    bool IsDevicePresent(uint8_t addr) {
        // 创建设备句柄
        i2c_master_dev_handle_t dev_handle;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        
        esp_err_t ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            return false;
        }
        
        // 发送一个字节的数据来检测设备
        uint8_t test_data = 0x00;
        ret = i2c_master_transmit(dev_handle, &test_data, 1, 100);
        
        // 删除设备句柄
        i2c_master_bus_rm_device(dev_handle);
        
        return (ret == ESP_OK);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        // Long press 4 seconds -> enter Wi-Fi configuration (re-provision)
        boot_button_.OnLongPress([this]() {
            ESP_LOGI(TAG, "Long press detected, entering Wi-Fi config mode (re-provisioning)");
            ResetWifiConfiguration();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#if CONFIG_LINGXI_ANIMA_UI
        display_ = new AnimaDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#elif CONFIG_XIAOZHI_DEFAULT_UI
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = font_emoji_64_init(),
#endif
                                    });
#endif
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
            .int_gpio_num = GPIO_NUM_NC, 
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,
                .mirror_x = 1,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        tp_io_config.scl_speed_hz = 400000;

        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);
        assert(tp);

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(), 
            .handle = tp,
        };

        lvgl_port_add_touch(&touch_cfg);
    }

    void InitializeCamera() {
        // Open camera power
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;  // LEDC通道选择  用于生成XCLK时钟 但是S3不用
        config.ledc_timer = LEDC_TIMER_2; // LEDC timer选择  用于生成XCLK时钟 但是S3不用
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;   // 这里写-1 表示使用已经初始化的I2C接口
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_HVGA;  // 480x320 instead of 640x480 to save memory
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }
    
    void InitializeImu() {
        // 仅初始化IMU硬件
        imu_ = new Qmi8658(i2c_bus_);
        
        if (imu_->Initialize() == ESP_OK) {
            ESP_LOGI(TAG, "IMU initialized successfully");
        } else {
            ESP_LOGW(TAG, "Failed to initialize IMU");
            delete imu_;
            imu_ = nullptr;
        }
    }

    void InitializePca9685() {
        IsDevicePresent(PCA9685_DEFAULT_ADDR);
        ESP_LOGI(TAG, "Initializing PCA9685 at address 0x40...");
        pca9685_ = new Pca9685(i2c_bus_, PCA9685_DEFAULT_ADDR);
        
        ESP_LOGI(TAG, "🔧 设置PCA9685 PWM频率为200Hz (适配DRV8837直流马达驱动)");
        esp_err_t ret = pca9685_->Initialize(1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PCA9685: %s", esp_err_to_name(ret));
            delete pca9685_;
            pca9685_ = nullptr;
        }
    }
    
    void InitializeVibration() {
        if (pca9685_ == nullptr) {
            ESP_LOGW(TAG, "PCA9685 not available, skipping vibration initialization");
            return;
        }
        
        vibration_skill_ = new Vibration(pca9685_, 0);
        
        esp_err_t ret = vibration_skill_->Initialize();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Vibration skill initialized successfully");
        } else {
            ESP_LOGE(TAG, "Failed to initialize vibration skill: %s", esp_err_to_name(ret));
            delete vibration_skill_;
            vibration_skill_ = nullptr;
        }
    }
    
    void InitializeMotion() {
        if (pca9685_ == nullptr) {
            ESP_LOGW(TAG, "PCA9685 not available, skipping motion initialization");
            return;
        }
        
        // 使用PCA9685的通道1和通道2连接DRV883X的IN1和IN2
        motion_skill_ = new Motion(pca9685_, 1, 2);
        
        esp_err_t ret = motion_skill_->Initialize();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Motion skill initialized successfully");
        } else {
            ESP_LOGE(TAG, "Failed to initialize motion skill: %s", esp_err_to_name(ret));
            delete motion_skill_;
            motion_skill_ = nullptr;
        }
    }
    
    void StartVibrationTask() {
        if (vibration_skill_) {
            esp_err_t ret = vibration_skill_->StartTask();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Vibration task started successfully");
            } else {
                ESP_LOGE(TAG, "Failed to start vibration task: %s", esp_err_to_name(ret));
            }
            // 如有振动效果测试需要再启用
            // vibration_skill_->EnableButtonTest(VIBRATION_SHORT_BUZZ, true); 
            ESP_LOGI(TAG, "Button cycle test enabled - press GPIO11 to test all patterns");
        }
    }
    
    void StartMotionTask() {
        if (motion_skill_) {
            esp_err_t ret = motion_skill_->StartTask();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Motion task started successfully");
            } else {
                ESP_LOGE(TAG, "Failed to start motion task: %s", esp_err_to_name(ret));
            }
        }
    }
    
    void InitializeInteractionSystem() {
        // 创建事件引擎
        event_engine_ = new EventEngine();
        // 初始化多点触摸引擎
        event_engine_->InitializeMultitouchEngine(i2c_bus_);
        // 初始化运动引擎（如果IMU可用）
        if (imu_) {
            event_engine_->InitializeMotionEngine(imu_, true);  // 启用调试输出
            // 当前功能模块固定，必须初始化成功
            // 重新加载配置，因为motion engine在Initialize()时还不存在
            // event_engine_->ReloadMotionConfig();
        }

        event_engine_->Initialize();

        // 初始化情感引擎
        event_engine_->InitializeEmotionEngine();

        // 创建事件上传器
        event_uploader_ = new EventUploader();
        event_uploader_->Enable(true);
        ESP_LOGI(TAG, "EventUploader created and enabled");

        // 设置情感状态上报回调
        event_engine_->SetEmotionReportCallback([this](const Event& event, float valence, float arousal) {
            ESP_LOGI(TAG, "🎭 Emotion state changed: V=%.2f, A=%.2f for event type=%d",
                     valence, arousal, (int)event.type);

            // 将情感状态设置到事件上传器中，用于云端上报
            if (event_uploader_) {
                event_uploader_->SetCurrentEmotionState(valence, arousal);
            }
        });

        // 事件处理策略已通过配置文件自动加载
        // 如需覆盖特定策略，可在此处调用：
        // event_engine_->ConfigureEventProcessing(EventType::TOUCH_TAP, custom_config);

        // 设置单事件回调（用于本地响应和状态更新）
        event_engine_->RegisterCallback([this](const Event& event) {
            // 1. 先执行本地响应（最高优先级，即时反应）
            if (local_response_controller_) {
                local_response_controller_->ProcessEvent(event);
            }

            // 2. 处理事件日志和情感状态更新
            HandleEvent(event);
        });

        // 设置批量事件回调（用于云端上传）
        event_engine_->RegisterBatchCallback([this](const std::vector<Event>& events) {
            ESP_LOGI(TAG, "Batch upload: processing %u events", events.size());

            // 批量上传事件到云端（真正的批量，一个JSON payload）
            if (event_uploader_) {
                event_uploader_->HandleBatchEvents(events);
            }
        });

        // 创建定时器，每50ms处理一次事件
        esp_timer_create_args_t event_timer_args = {};
        event_timer_args.callback = [](void* arg) {
            auto* engine = static_cast<EventEngine*>(arg);
            engine->Process();
        };
        event_timer_args.arg = event_engine_;
        event_timer_args.dispatch_method = ESP_TIMER_TASK;
        event_timer_args.name = "event_timer";
        event_timer_args.skip_unhandled_events = true;

        esp_timer_create(&event_timer_args, &event_timer_);
        esp_timer_start_periodic(event_timer_, 50000);  // 50ms

        ESP_LOGI(TAG, "Interaction system initialized and started");
    }
    
    void InitialSDCard() {
        sdhccard = SDmoduleInit();
        SDLoadImageTest();
        //sdhccard->TestFile();
    }

    void InitialAngleSensor() {
        angle_sensor = new AngleSensor(motion_skill_);
        set_anglehd(angle_sensor);

        as5600 = new As5600Sensor(i2c_bus_);
        set_as5600hd(as5600);
        BLDCModuleInit(as5600, pca9685_);
        pca9557_->SetOutputState(4, 1);
    }

    void InitializeAdcSample() {
        //DRV_AdcInit();
    }

    void InitializeMcpTools() {
        ESP_LOGI(TAG, "Initializing MCP local response tools...");
        
        try {
            // 创建McpResponseController实例
            mcp_response_controller_ = new McpResponseController(
                motion_skill_,
                vibration_skill_,
                event_engine_,
                [this]() -> Display* { return GetDisplay(); },
                [this]() -> std::string { return GetCurrentAnimation(); },
                [this](const std::string& animation) { SetCurrentAnimation(animation); }
            );
            
            // 初始化MCP工具
            if (mcp_response_controller_->Initialize()) {
                ESP_LOGI(TAG, "✅ MCP local response system initialized successfully");
            } else {
                ESP_LOGE(TAG, "❌ Failed to initialize MCP local response system");
                delete mcp_response_controller_;
                mcp_response_controller_ = nullptr;
            }
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Exception during MCP tools initialization: %s", e.what());
            if (mcp_response_controller_) {
                delete mcp_response_controller_;
                mcp_response_controller_ = nullptr;
            }
        }
    }
    
    void InitializeLocalResponseSystem() {
        ESP_LOGI(TAG, "Initializing Local Response System...");
        
        try {
            // 创建本地响应控制器
            local_response_controller_ = new LocalResponseController(
                motion_skill_,
                vibration_skill_, 
                GetDisplay()
            );
            
            // 初始化本地响应系统
            if (local_response_controller_->Initialize()) {
                ESP_LOGI(TAG, "✅ Local Response System initialized successfully");
                // 注册设备状态变化监听器，用于状态响应
                DeviceStateEventManager::GetInstance().RegisterStateChangeCallback(
                    [this](DeviceState previous_state, DeviceState current_state) {
                        (void)previous_state;
                        if (local_response_controller_) {
                            local_response_controller_->ProcessStateChange(current_state);
                        }
                    }
                );
                ESP_LOGI(TAG, "✅ Device state change listener registered");
            } else {
                ESP_LOGE(TAG, "❌ Failed to initialize Local Response System");
                delete local_response_controller_;
                local_response_controller_ = nullptr;
            }
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Exception during Local Response System initialization: %s", e.what());
            if (local_response_controller_) {
                delete local_response_controller_;
                local_response_controller_ = nullptr;
            }
        }
    }
    
    void HandleEvent(const Event& event) {
        // 在终端输出事件信息
        const char* event_name = "";
        const ImuData& data = event.data.imu_data;
        
        switch (event.type) {
            case EventType::MOTION_FREE_FALL:
                event_name = "FREE_FALL";
                ESP_LOGW(TAG, "⚠️ FREE FALL DETECTED! Accel magnitude: %.3f g", 
                        std::sqrt(data.accel_x * data.accel_x + 
                                data.accel_y * data.accel_y + 
                                data.accel_z * data.accel_z));
                break;
            case EventType::MOTION_SHAKE_VIOLENTLY:
                event_name = "SHAKE_VIOLENTLY";
                ESP_LOGW(TAG, "⚡ VIOLENT SHAKE! Accel: X=%.2f Y=%.2f Z=%.2f g", 
                        data.accel_x, data.accel_y, data.accel_z);
                break;
            case EventType::MOTION_FLIP: 
                event_name = "FLIP";
                ESP_LOGI(TAG, "🔄 Device flipped! (gyro: x=%.1f y=%.1f z=%.1f deg/s)", 
                        data.gyro_x, data.gyro_y, data.gyro_z);
                break;
            case EventType::MOTION_SHAKE: 
                event_name = "SHAKE";
                ESP_LOGI(TAG, "🔔 Device shaken!");
                break;
            case EventType::MOTION_PICKUP: 
                event_name = "PICKUP";
                ESP_LOGI(TAG, "📱 Device picked up!");
                break;
            case EventType::MOTION_UPSIDE_DOWN:
                event_name = "UPSIDE_DOWN";
                ESP_LOGI(TAG, "🙃 Device is upside down! (Z-axis: %.2f g)", data.accel_z);
                break;
            // 处理触摸事件
            case EventType::TOUCH_TAP: {
                event_name = "TOUCH_TAP";
                // 使用新的TouchEventData结构
                const char* side_str = "UNKNOWN";
                switch (event.data.touch_data.position) {
                    case TouchPosition::LEFT: side_str = "LEFT"; break;
                    case TouchPosition::RIGHT: side_str = "RIGHT"; break;
                    case TouchPosition::BOTH: side_str = "BOTH"; break;
                    case TouchPosition::ANY: side_str = "ANY"; break;
                }
                ESP_LOGI(TAG, "👆 Touch TAP on %s side! (duration: %lu ms, count: %lu)", 
                        side_str,
                        (unsigned long)event.data.touch_data.duration_ms,
                        (unsigned long)event.data.touch_data.tap_count);
                break;
            }
            case EventType::TOUCH_DOUBLE_TAP:
                event_name = "TOUCH_DOUBLE_TAP";
                ESP_LOGI(TAG, "👆👆 Touch DOUBLE TAP! (duration: %lu ms)", 
                        (unsigned long)event.data.touch_data.duration_ms);
                break;
            case EventType::TOUCH_LONG_PRESS:
                event_name = "TOUCH_LONG_PRESS";
                {
                    const char* press_side = "UNKNOWN";
                    switch (event.data.touch_data.position) {
                        case TouchPosition::LEFT: press_side = "LEFT"; break;
                        case TouchPosition::RIGHT: press_side = "RIGHT"; break;
                        case TouchPosition::BOTH: press_side = "BOTH"; break;
                        case TouchPosition::ANY: press_side = "ANY"; break;
                    }
                    ESP_LOGI(TAG, "👇 Touch LONG PRESS on %s side! (duration: %lu ms)", 
                            press_side,
                            (unsigned long)event.data.touch_data.duration_ms);
                }
                break;
            
            // 其他触摸事件
            case EventType::TOUCH_CRADLED:
                event_name = "TOUCH_CRADLED";
                ESP_LOGI(TAG, "🤗 Device is being cradled!");
                break;
            case EventType::TOUCH_TICKLED:
                event_name = "TOUCH_TICKLED";
                ESP_LOGI(TAG, "😄 Device is being tickled!");
                break;
            case EventType::TOUCH_HOLD:
            case EventType::TOUCH_RELEASE:
                // 暂时不处理
                return;
                
            // 音频事件（预留）
            case EventType::AUDIO_WAKE_WORD:
            case EventType::AUDIO_SPEAKING:
            case EventType::AUDIO_LISTENING:
                // 暂时不处理
                return;
                
            // 系统事件（预留）
            case EventType::SYSTEM_BOOT:
            case EventType::SYSTEM_SHUTDOWN:
            case EventType::SYSTEM_ERROR:
                // 暂时不处理
                return;
                
            // 默认情况
            case EventType::MOTION_NONE:
            default: 
                return;
        }
        
        // 显示详细的IMU数据
        ESP_LOGD(TAG, "IMU Event [%s] - Accel(g): X=%.2f Y=%.2f Z=%.2f | Angles(°): X=%.1f Y=%.1f Z=%.1f",
                event_name,
                data.accel_x, data.accel_y, data.accel_z,
                data.angle_x, data.angle_y, data.angle_z);
    }

    void TestHumanFaceModel() {
        const char* filepath = "/sdcard/face_dectect_models/test_face.jpg";
        FILE *f = fopen(filepath, "r");
        if (f == NULL) {
            ESP_LOGW(TAG, "TestHumanFaceModel jpg file err: %s", filepath);
            return;
        }
        // 获取文件大小置位到文件起始处
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        ESP_LOGI(TAG, "jpg file size: %ld", file_size);
        fseek(f, 0, SEEK_SET);
        char* databuf = (char* )malloc((file_size) * sizeof(char));
        fread(databuf, 1, file_size, f);
        fclose(f); // 关闭文件

        dl::image::jpeg_img_t jpeg_img = {.data = static_cast<void*>(databuf),
                                      .data_len = static_cast<size_t>(file_size)};
        auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
        HumanFaceDetect *detect = new HumanFaceDetect();
        auto &detect_results = detect->run(img);
        for (const auto &res : detect_results) {
            ESP_LOGI(TAG,
                    "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                    res.score,
                    res.box[0],
                    res.box[1],
                    res.box[2],
                    res.box[3]);
            ESP_LOGI(
                TAG,
                "left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], right_eye: [%d, %d], right_mouth: [%d, %d]]",
                res.keypoint[0],
                res.keypoint[1],
                res.keypoint[2],
                res.keypoint[3],
                res.keypoint[4],
                res.keypoint[5],
                res.keypoint[6],
                res.keypoint[7],
                res.keypoint[8],
                res.keypoint[9]);
        }
        delete detect;
        heap_caps_free(img.data);
        
        free(databuf);

    }
public:
    ALichuangTest() : boot_button_(BOOT_BUTTON_GPIO, false, 4000 /* ms */) {
        InitializeAdcSample();
        vTaskDelay(pdMS_TO_TICKS(10));
        InitialSDCard();
        vTaskDelay(pdMS_TO_TICKS(10));
        InitializeI2c();
        InitializeSpi();
        vTaskDelay(pdMS_TO_TICKS(10));
        InitializeSt7789Display();
        //InitializeTouch();
        InitializeButtons();
        InitializeCamera();
        InitializeImu();  // 初始化IMU硬件
        InitializePca9685();  // 初始化PCA9685 PWM控制器
        InitializeVibration();  // 初始化振动技能（使用PCA9685）
        InitializeMotion();  // 初始化直流马达动作控制技能
        InitialAngleSensor();
        InitializeInteractionSystem();  // 初始化交互系统

        GetBacklight()->RestoreBrightness();
#if CONFIG_LINGXI_ANIMA_UI
        // 启动GIF动画播放任务
        StartAnimationPlay();
#endif
        // 启动振动任务
        StartVibrationTask();
        // 启动直流马达动作控制任务
        StartMotionTask();
                
        // 初始化本地响应系统（在交互系统和技能初始化后）
        InitializeLocalResponseSystem();

        // 所有skills初始化完成后，初始化MCP工具
        InitializeMcpTools();

#if !CONFIG_DISABLE_BOOT_FACE_DETECTION_TEST
        // 测试人脸检测模型是否工作
        // 注意: 此测试会增加启动时间(~2s)和峰值内存(~800KB)
        // 建议在生产环境中通过 menuconfig 禁用此选项
        TestHumanFaceModel();
#else
        ESP_LOGI(TAG, "Boot face detection test disabled (saves ~2s startup + ~800KB peak memory)");
#endif
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(
            i2c_bus_, 
            pca9557_);
        return &audio_codec;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

#if CONFIG_LINGXI_ANIMA_UI
    virtual AnimaDisplay* GetDisplay() override {
        return display_;
    }
#elif CONFIG_XIAOZHI_DEFAULT_UI
    virtual Display* GetDisplay() override {
        return display_;
    }
#endif
    
    virtual std::string GetBoardType() override {
        return "lingxi";
    }
    
    // 获取运动检测器（可选，用于外部访问）
    EventEngine* GetEventEngine() {
        return event_engine_;
    }
    
    // 获取事件上传器（供Application使用）
    EventUploader* GetEventUploader() { 
        return event_uploader_; 
    }
    
    // 获取IMU（可选，用于外部访问）
    Qmi8658* GetImu() {
        return imu_;
    }
    
    // 获取振动技能管理器（可选，用于外部访问和测试）
    Vibration* GetVibration() {
        return vibration_skill_;
    }
    
    // 获取直流马达动作控制技能（可选，用于外部访问和测试）
    Motion* GetMotion() {
        return motion_skill_;
    }
    
    // 获取本地响应控制器（用于调试和测试）
    LocalResponseController* GetLocalResponseController() {
        return local_response_controller_;
    }

#if CONFIG_ENABLE_BLUETOOTH_PROVISIONING
public:
    virtual void StartNetwork() override {
#ifdef CONFIG_BLUETOOTH_PROVISIONING_ALWAYS_ON_BOOT
        // Testing mode: always start BluFi provisioning on boot
        auto& app = Application::GetInstance();
        // Free RAM before enabling BT: stop audio processing temporarily
        app.GetAudioService().Stop();
        app.SetDeviceState(kDeviceStateWifiConfiguring);

        auto& blufi = BlufiProvisioning::GetInstance();

        blufi.OnConfigured([&](const std::string& ssid, const std::string& password){
            ESP_LOGI(TAG, "BluFi configured: SSID=%s", ssid.c_str());
            vTaskDelay(pdMS_TO_TICKS(1500));
            esp_restart();
        });

        blufi.Start();

        int timeout_sec = CONFIG_BLUETOOTH_PROVISIONING_TIMEOUT;
        if (!blufi.WaitForConfigured(timeout_sec * 1000)) {
            ESP_LOGW(TAG, "BluFi provisioning timeout, stopping");
            blufi.Stop();
            // Resume audio on fallback
            app.GetAudioService().Start();
#if CONFIG_BLUFI_FALLBACK_TO_WEB_CONFIG
            WifiBoard::StartNetwork();
            return;
#endif
        }
        return;
#else
        // Default behavior: use standard WiFi connection flow
        WifiBoard::StartNetwork();
#endif
    }
#endif
    
};

DECLARE_BOARD(ALichuangTest);
