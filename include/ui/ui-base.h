#ifndef UI_BASE_H
#define UI_BASE_H

#include "globals.h"
#include "robot-config.h"
#include "vex.h"

#include <functional>
#include <string>
using namespace vex;

std::string float_to_string(double num, int decimal_digits = 3);

class Label {
  protected:
    color current_fill_color_;
    int x_, y_, width_, height_;
    std::string text_;

  public:
    Label(const int& x, const int& y, const int& width, const int& height,
          const std::string& text);
    Label(const int& x, const int& y, const int& width, const int& height,
          const float& num);
    void setText(const std::string& text) { text_ = text; }
    void setText(const float& num) { text_ = float_to_string(num); }
    color getCurrentFillColor() { return current_fill_color_; }
    void render(const color& text_color, const color& border_color,
                const color& fill_color, const fontType font = mono20);
};

class Button : public Label {
  private:
    bool prev_overall_press_ = false;
    bool detecting_ = false;
    bool prev_in_ = false;
    bool in();

    std::function<void()> callback_ = NULL;

  public:
    Button(const int& x, const int& y, const int& width, const int& height,
           const std::string& text, const std::function<void()>& callback);
    color getCurrentFillColor() { return current_fill_color_; }
    void render(const color& text_color, const color& border_color,
                const color& fill_color, const fontType font = mono20);
    void check();
};

#endif // UI_BASE_H
