#ifndef UI_BASE_H
#define UI_BASE_H

#include "globals.h"
#include "robot-config.h"
#include "vex.h"
#include <functional>
#include <string>

using namespace vex;

/**
 * @brief 浮点数转字符串
 * @param num 输入数值
 * @param decimal_digits 保留小数位数
 * @return 格式化后的字符串
 */
std::string float_to_string(double num, int decimal_digits = 3);

/**
 * @brief 屏幕界面枚举
 */
enum Interface { OPTIONS, MOTORS, DEVICES, INFO, PARAMS };
extern Interface current_interface;

/**
 * @brief 参数页配置档位
 */
enum ParamProfile { PROFILE_REACH, PROFILE_TRACE };
extern ParamProfile current_param_profile;

/**
 * @brief 刷新 UI 状态
 */
void refresh();
/**
 * @brief 切换当前显示界面
 * @param interface 目标界面
 */
void setInterface(Interface interface);
/**
 * @brief 设置当前自动路线
 * @param route 目标路线
 */
void setRoute(Route route);
/**
 * @brief 设置当前颜色
 * @param color 目标颜色
 */
void setColor(Color color);
/**
 * @brief 设置当前模式
 * @param mode 目标模式
 */
void setMode(Mode mode);
/**
 * @brief 设置当前参数配置档位
 * @param profile 目标档位
 */
void setParamProfile(ParamProfile profile);

/**
 * @brief UI 标签组件
 */
class Label {
  protected:
    int x_, y_, width_, height_;
    std::string text_;

  public:
    /**
     * @brief 构造文本标签
     */
    Label(const int& x, const int& y, const int& width, const int& height,
          const std::string& text);
    /**
     * @brief 构造数值标签
     */
    Label(const int& x, const int& y, const int& width, const int& height,
          const float& num);
    void setText(const std::string& text) { text_ = text; }
    void setText(const float& num) { text_ = float_to_string(num); }
    /**
     * @brief 渲染标签
     */
    void render(const color& text_color = color(128, 128, 128),
                const color& border_color = color::white,
                const color& fill_color = color::white, const fontType font = mono20);
};

/**
 * @brief UI 按钮组件
 */
class Button : public Label {
  private:
    bool last_overall_press_ = false;
    bool detecting_ = false;
    bool last_in_ = false;
    bool in();

    std::function<void()> callback_ = NULL;

  public:
    /**
     * @brief 构造按钮
     */
    Button(const int& x, const int& y, const int& width, const int& height,
           const std::string& text, const std::function<void()>& callback);

    /**
     * @brief 渲染按钮
     */
    void render(const color& text_color = color::black,
                const color& border_color = color::black,
                const color& fill_color = color::white, const fontType font = mono20);
    /**
     * @brief 检查并触发按钮回调
     */
    void check();
};

#endif // UI_BASE_H
