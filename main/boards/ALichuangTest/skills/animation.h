#ifndef ANIMA_DISPLAY_H
#define ANIMA_DISPLAY_H
#include "display/lcd_display.h"
#include <functional>
#include <unordered_map>
#include <string>

LV_IMAGE_DECLARE(system_boot_up);        // 启动动画

// 前向声明
enum class EmotionQuadrant;

class AnimaDisplay : public LcdDisplay {
public:
    AnimaDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy);
    virtual ~AnimaDisplay();

    // 动画变化回调
    virtual void OnAnimationChanged(std::function<void(const std::string&)> callback) {
        animation_callback_ = callback;
    }

    // 动画/情感显示接口实现
    virtual void SetEmotion(const char* emotion) override;  // 兼容接口：触发动画回调
    virtual void SetAnima(const std::string& animation) override;  // 新接口：直接播放GIF动画(默认播放一次)
    virtual void SetAnima(const std::string& animation, int loop_count) override;  // 重载：指定播放次数(loop_count<0为无限循环)

    // UI方法重载为空 - 因为使用GIF替代传统UI
    virtual void SetStatus(const char* status) override {}
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override {}
    virtual void SetChatMessage(const char* role, const char* content) override {}
    virtual void SetPreviewImage(std::unique_ptr<LvglImage> image) override {}
    virtual void UpdateStatusBar(bool update_all = false) override {}
    virtual void SetPowerSaveMode(bool on) override {}
    virtual void SetTheme(Theme* theme) override;

    // 静态工具函数：根据情感象限获取对应的idle动画名称
    static std::string GetIdleAnimationByQuadrant(EmotionQuadrant quadrant);

    // 获取开机动画开始时间（微秒）
    static int64_t GetBootAnimationStartTime() { return boot_animation_start_time_; }

private:
    // 动画名称到文件路径的哈希映射表
    static const std::unordered_map<std::string, const char*> animation_maps_;
    static int64_t boot_animation_start_time_;  // 开机动画开始时间（微秒）
    lv_obj_t* animation_gif_;  // GIF动画组件

protected:
    // 重载SetupUI为简化版本，避免复杂UI初始化
    void SetupUI();

    // 动画变化回调函数
    std::function<void(const std::string&)> animation_callback_ = nullptr;
};
#endif
