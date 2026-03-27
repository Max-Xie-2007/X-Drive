#ifndef UI_BASE_H
#define UI_BASE_H

#include "basic/robot-config.h"
#include "vex.h"
#include <functional>
#include <string>

using namespace vex;

std::string float_to_string(double num, int decimal_digits = 3);

enum Interface { OPTIONS, MOTORS, DEVICES, INFO, PARAMS };
extern Interface current_interface;

enum ParamProfile {
    PROFILE_FWD,
    PROFILE_TURN,
    PROFILE_POSE_FAR,
    PROFILE_POSE_CLOSE_BAD,
    PROFILE_POSE_CLOSE_GOOD
};
extern ParamProfile current_param_profile;

void refresh();
void setInterface(Interface interface);
void setRoute(Route route);
void setColor(Color color);
void setMode(Mode mode);
void setParamProfile(ParamProfile profile);

class Label {
  protected:
    int x_, y_, width_, height_;
    std::string text_;

  public:
    Label(const int& x, const int& y, const int& width, const int& height,
          const std::string& text);
    Label(const int& x, const int& y, const int& width, const int& height,
          const float& num);
    void setText(const std::string& text) { text_ = text; }
    void setText(const float& num) { text_ = float_to_string(num); }
    void render(const color& text_color = color(128, 128, 128),
                const color& border_color = color::white,
                const color& fill_color = color::white, const fontType font = mono20);
};

class Button : public Label {
  private:
    bool last_overall_press_ = false;
    bool detecting_ = false;
    bool last_in_ = false;
    bool in();

    std::function<void()> callback_ = NULL;

  public:
    Button(const int& x, const int& y, const int& width, const int& height,
           const std::string& text, const std::function<void()>& callback);

    void render(const color& text_color = color::black,
                const color& border_color = color::black,
                const color& fill_color = color::white, const fontType font = mono20);
    void check();
};

#endif // UI_BASE_H
