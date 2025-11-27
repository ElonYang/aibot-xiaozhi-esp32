#include "animation.h"

#include <esp_log.h>
#include <esp_err.h>
#include <esp_lvgl_port.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cstring>

#include "board.h"
// Use correct relative path to emotion engine header
#include "../interaction/core/emotion_engine.h"

#define TAG "AnimaDisplay"

#define BOOT_GIF_PATH  "/sdcard/system/boot_up/system_boot_up.gif"
#define DEFAULT_GIF_PATH "/sdcard/state_expression/idle/idle_q1/idle_q1.gif"

// 静态变量初始化
int64_t AnimaDisplay::boot_animation_start_time_ = 0;

// 动画名称到文件路径的哈希映射表，O(1) 查找时间复杂度
const std::unordered_map<std::string, const char*> AnimaDisplay::animation_maps_ = {
    {"motion_flip",    "/sdcard/emergency/motion_upside_down/motion_upside_down.gif"},
    {"motion_free_fall",    "/sdcard/emergency/motion_shake_violently/motion_shake_violently.gif"},
    {"motion_shake_violently",    "/sdcard/emergency/motion_shake_violently/motion_shake_violently.gif"},
    {"motion_upside_down",    "/sdcard/emergency/motion_upside_down/motion_upside_down.gif"},
    {"motion_pickup_q1",    "/sdcard/interaction/motion_pickup_q1/motion_pickup_q1.gif"},
    {"motion_pickup_q2",    "/sdcard/interaction/motion_pickup_q2/motion_pickup_q2.gif"},
    {"motion_pickup_q3",    "/sdcard/interaction/motion_pickup_q3/motion_pickup_q3.gif"},
    {"motion_pickup_q4",    "/sdcard/interaction/motion_pickup_q4/motion_pickup_q4.gif"},
    {"motion_shake_q1",    "/sdcard/interaction/motion_shake_q1/motion_shake_q1.gif"},
    {"motion_shake_q2",    "/sdcard/interaction/motion_shake_q2/motion_shake_q2.gif"},
    {"motion_shake_q3",    "/sdcard/interaction/motion_shake_q3/motion_shake_q3.gif"},
    {"motion_shake_q4",    "/sdcard/interaction/motion_shake_q4/motion_shake_q4.gif"},
    {"touch_cradled_q1",    "/sdcard/interaction/touch_cradled_q1/touch_cradled_q1.gif"},
    {"touch_cradled_q2",    "/sdcard/interaction/touch_cradled_q2/touch_cradled_q2.gif"},
    {"touch_cradled_q3",    "/sdcard/interaction/touch_cradled_q3/touch_cradled_q3.gif"},
    {"touch_cradled_q4",    "/sdcard/interaction/touch_cradled_q4/touch_cradled_q4.gif"},
    {"touch_long_press_q1",    "/sdcard/interaction/touch_long_press_q1/touch_long_press_q1.gif"},
    {"touch_long_press_q2",    "/sdcard/interaction/touch_long_press_q2/touch_long_press_q2.gif"},
    {"touch_long_press_q3",    "/sdcard/interaction/touch_long_press_q3/touch_long_press_q3.gif"},
    {"touch_long_press_q4",    "/sdcard/interaction/touch_long_press_q4/touch_long_press_q4.gif"},
    {"touch_tap_q1",    "/sdcard/interaction/touch_tap_q1/touch_tap_q1.gif"},
    {"touch_tap_q2",    "/sdcard/interaction/touch_tap_q2/touch_tap_q2.gif"},
    {"touch_tap_q3",    "/sdcard/interaction/touch_tap_q3/touch_tap_q3.gif"},
    {"touch_tap_q4",    "/sdcard/interaction/touch_tap_q4/touch_tap_q4.gif"},
    {"touch_tickled_q1",    "/sdcard/interaction/touch_tickled_q1/touch_tickled_q1.gif"},
    {"touch_tickled_q2",    "/sdcard/interaction/touch_tickled_q2/touch_tickled_q2.gif"},
    {"touch_tickled_q3",    "/sdcard/interaction/touch_tickled_q3/touch_tickled_q3.gif"},
    {"touch_tickled_q4",    "/sdcard/interaction/touch_tickled_q4/touch_tickled_q4.gif"},
    {"idle_q1", "/sdcard/state_expression/idle/idle_q1/idle_q1.gif"},
    {"idle_q2", "/sdcard/state_expression/idle/idle_q2/idle_q2.gif"},
    {"idle_q3", "/sdcard/state_expression/idle/idle_q3/idle_q3.gif"},
    {"sleepy", "/sdcard/state_expression/idle/idle_q3/idle_q3.gif"},
    {"idle_q4", "/sdcard/state_expression/idle/idle_q4/idle_q4.gif"},
    {"listening_q1", "/sdcard/state_expression/listening/listening_q1/listening_q1.gif"},
    {"listening_q2", "/sdcard/state_expression/listening/listening_q2/listening_q2.gif"},
    {"listening_q3", "/sdcard/state_expression/listening/listening_q3/listening_q3.gif"},
    {"listening_q4", "/sdcard/state_expression/listening/listening_q4/listening_q4.gif"},
    // 思考状态动画
    {"thinking", "/sdcard/state_expression/thinking/thinking_q1/thinking_q1.gif"},
    // 8种说话表情动画（与TTS配合使用）
    {"calm",    "/sdcard/state_expression/speaking/talk_calm/talk_calm.gif"},
    {"happy",   "/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"laughing","/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"sad",     "/sdcard/state_expression/speaking/talk_sad/talk_sad.gif"},
    {"angry",   "/sdcard/state_expression/speaking/talk_angry/talk_angry.gif"},
    {"scared",  "/sdcard/state_expression/speaking/talk_scared/talk_scared.gif"},
    {"curious", "/sdcard/state_expression/speaking/talk_curious/talk_curious.gif"},
    {"shy",     "/sdcard/state_expression/speaking/talk_shy/talk_shy.gif"},
    {"content", "/sdcard/state_expression/speaking/talk_content/talk_content.gif"},
    // 常见同义词映射到现有说话表情
    {"funny",   "/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"loving",  "/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"confident","/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"delicious","/sdcard/state_expression/speaking/talk_happy/talk_happy.gif"},
    {"crying",  "/sdcard/state_expression/speaking/talk_sad/talk_sad.gif"},
    {"silly",   "/sdcard/state_expression/speaking/talk_sad/talk_sad.gif"},
    {"confused","/sdcard/state_expression/speaking/talk_curious/talk_curious.gif"},
    {"surprised","/sdcard/state_expression/speaking/talk_curious/talk_curious.gif"},
    {"shocked", "/sdcard/state_expression/speaking/talk_scared/talk_scared.gif"},
    {"embarrassed","/sdcard/state_expression/speaking/talk_shy/talk_shy.gif"},
    {"relaxed", "/sdcard/state_expression/speaking/talk_content/talk_content.gif"},
    {"winking", "/sdcard/state_expression/speaking/talk_content/talk_content.gif"},
    // 系统提示/图标别名
    {"system_boot_up", "/sdcard/system/boot_up/system_boot_up.gif"},
    {"link",     "/sdcard/state_expression/thinking/thinking_q1/thinking_q1.gif"},
    {"circle_xmark", "/sdcard/state_expression/thinking/thinking_q1/thinking_q1.gif"},
    {"microchip_ai", BOOT_GIF_PATH},
};

// 根据情感象限获取对应的idle动画名称
std::string AnimaDisplay::GetIdleAnimationByQuadrant(EmotionQuadrant quadrant) {
    switch (quadrant) {
        case EmotionQuadrant::POSITIVE_HIGH_AROUSAL:
            return "idle_q1";  // 积极高激活
        case EmotionQuadrant::NEGATIVE_HIGH_AROUSAL:
            return "idle_q2";  // 消极高激活
        case EmotionQuadrant::NEGATIVE_LOW_AROUSAL:
            return "idle_q3";  // 消极低激活
        case EmotionQuadrant::POSITIVE_LOW_AROUSAL:
            return "idle_q4";  // 积极低激活
        default:
            return "idle_q1";  // 默认返回Q1
    }
}

AnimaDisplay::AnimaDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                           int width, int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y, bool swap_xy)
    : LcdDisplay(panel_io, panel, width, height), animation_gif_(nullptr) {

    // 不绘制白屏，让开机动画作为第一个显示内容
    // 注释掉白屏绘制，避免闪烁
    // std::vector<uint16_t> buffer(width_, 0xFFFF);
    // for (int y = 0; y < height_; y++) {
    //     esp_lcd_panel_draw_bitmap(panel_, 0, y, width_, y + 1, buffer.data());
    // }

    // Set the display to on
    ESP_LOGD(TAG, "Turning display on");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    ESP_LOGD(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGD(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    // 设置为 5ms 以支持高精度 GIF 播放
    // 这样 LVGL 定时器可以更频繁地检查帧切换，避免播放变慢
    // 25 FPS = 40ms per frame，5ms 定时器可以精确匹配（40ms = 8 ticks）
    port_cfg.timer_period_ms = 5;
    lvgl_port_init(&port_cfg);

    ESP_LOGD(TAG, "Adding LCD display");
    const lvgl_port_display_cfg_t display_cfg = {
        .io_handle = panel_io_,
        .panel_handle = panel_,
        .control_handle = nullptr,
        .buffer_size = static_cast<uint32_t>(width_ * 20),
        .double_buffer = false,
        .trans_size = 0,
        .hres = static_cast<uint32_t>(width_),
        .vres = static_cast<uint32_t>(height_),
        .monochrome = false,
        .rotation = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 0,
            .swap_bytes = 1,
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };

    display_ = lvgl_port_add_disp(&display_cfg);
    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    if (offset_x != 0 || offset_y != 0) {
        lv_display_set_offset(display_, offset_x, offset_y);
    }
    lv_fs_stdio_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    // 调用简化的SetupUI
    SetupUI();
}

AnimaDisplay::~AnimaDisplay() {
    // 在析构前先暂停GIF动画,防止timer回调访问已释放的对象
    if (animation_gif_ != nullptr) {
        DisplayLockGuard lock(this);
        lv_gif_pause(animation_gif_);
        lv_obj_delete(animation_gif_);
        animation_gif_ = nullptr;
    }
}

void AnimaDisplay::SetupUI() {
    DisplayLockGuard lock(this);
    
    // 创建最基本的LVGL对象以避免空指针，但不设置复杂UI
    auto screen = lv_screen_active();
    
    // 创建透明容器
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, width_, height_);
    lv_obj_set_style_bg_opa(container_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_pad_all(container_, 0, 0);
    
    // 创建基本的状态标签（设为隐藏，避免系统调用时崩溃）
    emoji_label_ = lv_label_create(container_);
    lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);

    animation_gif_ = lv_gif_create(screen);
    lv_obj_set_size(animation_gif_, width_, height_);
    lv_obj_set_style_border_width(animation_gif_, 0, 0);
    lv_obj_set_style_bg_opa(animation_gif_, LV_OPA_TRANSP, 0);
    lv_obj_center(animation_gif_);

    // 立即加载开机动画，尽早显示内容，避免空白屏
    // 音频会稍后在 Application::Start() 中播放（允许不完全同步）
    lv_gif_set_src(animation_gif_, &system_boot_up);

    // 检查 GIF 是否加载成功
    if (!lv_gif_is_loaded(animation_gif_)) {
        ESP_LOGE(TAG, "Failed to load boot animation GIF");
    } else {
        // 设置只播放一次：根据 SetAnima() 的逻辑，1 表示播放1次
        lv_gif_set_loop_count(animation_gif_, 1);
        ESP_LOGI(TAG, "Boot animation loop count set to 1");
        ESP_LOGI(TAG, "Boot animation configured for 25 FPS:");
        ESP_LOGI(TAG, "  - Frame delay: 40ms");
        ESP_LOGI(TAG, "  - LVGL port timer: 5ms");
        ESP_LOGI(TAG, "  - GIF internal timer: 5ms");
        ESP_LOGI(TAG, "  - Expected precise playback at 25 FPS");
    }

    // 记录动画开始时间
    boot_animation_start_time_ = esp_timer_get_time();

    ESP_LOGI(TAG, "Boot animation loaded and started at constructor (%.3f ms)",
             boot_animation_start_time_ / 1000.0);

    // 其他UI组件设为nullptr，避免系统调用时出错
    status_label_ = nullptr;
    notification_label_ = nullptr;
    network_label_ = nullptr;
    mute_label_ = nullptr;
    battery_label_ = nullptr;
    chat_message_label_ = nullptr;
    low_battery_popup_ = nullptr;
    low_battery_label_ = nullptr;
    status_bar_ = nullptr;
    content_ = nullptr;
    
    ESP_LOGD(TAG, "Simplified UI setup completed");
}

void AnimaDisplay::SetEmotion(const char* emotion) {
    // 触发动画变化回调
    if (animation_callback_) {
        animation_callback_(std::string(emotion));
    }
}


void AnimaDisplay::SetTheme(Theme* theme) {
    // AnimaDisplay doesn't use traditional themes since it uses GIF-based rendering
    // Store the theme but don't apply it to UI elements
    current_theme_ = theme;
}

void AnimaDisplay::SetAnima(const std::string& animation) {
    // 默认播放一次
    SetAnima(animation, 1);
}

void AnimaDisplay::SetAnima(const std::string& animation, int loop_count) {
    DisplayLockGuard lock(this);

    // 使用哈希表快速查找，时间复杂度 O(1)
    auto it = animation_maps_.find(animation);
    const char* gif_path = nullptr;

    if (it != animation_maps_.end()) {
        gif_path = it->second;
    } else {
        gif_path = DEFAULT_GIF_PATH;
        ESP_LOGW(TAG, "未知动画'%s'，使用默认动画", animation.c_str());
    }

    // 计算实际循环次数：LVGL中0通常表示无限循环
    // 兼容调用方传入的-1 或 0 代表无限循环
    int effective_loops = (loop_count <= 0) ? 0 : loop_count;

    // 若为说话类动画，强制改为无限循环
    if (gif_path && strstr(gif_path, "/state_expression/speaking/") != nullptr) {
        effective_loops = 0; // 无限循环
        ESP_LOGI(TAG, "设置动画: %s (循环次数: 无限, 说话模式)", animation.c_str());
    } else {
        ESP_LOGI(TAG, "设置动画: %s (循环次数: %s)",
                 animation.c_str(),
                 effective_loops == 0 ? "无限" : std::to_string(effective_loops).c_str());
    }

    // 完全重建GIF对象以避免timer竞态条件
    // 删除旧GIF对象(包括其内部timer)
    if (animation_gif_ != nullptr) {
        lv_obj_delete(animation_gif_);
        animation_gif_ = nullptr;
    }

    // 创建新的GIF对象
    auto screen = lv_screen_active();
    animation_gif_ = lv_gif_create(screen);
    if (animation_gif_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create new GIF object");
        return;
    }

    // 配置GIF对象
    lv_obj_set_size(animation_gif_, width_, height_);
    lv_obj_set_style_border_width(animation_gif_, 0, 0);
    lv_obj_set_style_bg_opa(animation_gif_, LV_OPA_TRANSP, 0);
    lv_obj_center(animation_gif_);

    // 设置GIF源
    lv_gif_set_src(animation_gif_, gif_path);

    // 检查GIF是否加载成功
    if (!lv_gif_is_loaded(animation_gif_)) {
        ESP_LOGE(TAG, "Failed to load GIF from path: %s", gif_path);
        // 即使失败也不删除对象，保持一个空的GIF对象
        return;
    }

    // 设置循环次数（0 表示无限循环）
    lv_gif_set_loop_count(animation_gif_, effective_loops);

    // GIF创建后会自动开始播放，不需要额外调用restart
    ESP_LOGD(TAG, "Animation set successfully: %s", animation.c_str());
}
