#include "ui/ui-base.h"
#include "ui/ui-brain.h"
#include "ui/ui-controller.h"

#include <iomanip>
#include <sstream>
#include <string>
std::string float_to_string(double num, int decimal_digits) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(decimal_digits) << num;
    return ss.str();
}

Label::Label(const int& x, const int& y, const int& width, const int& height,
             const std::string& text)
    : x_(x), y_(y), width_(width), height_(height), text_(text) {}

Label::Label(const int& x, const int& y, const int& width, const int& height,
             const float& num)
    : x_(x), y_(y), width_(width), height_(height), text_(float_to_string(num)) {}
void Label::render(const color& text_color, const color& border_color,
                   const color& fill_color, const fontType font) {
    current_fill_color_ = fill_color;
    Brain.Screen.setPenColor(border_color);
    Brain.Screen.setFillColor(fill_color);
    Brain.Screen.drawRectangle(x_, y_, width_, height_, fill_color);
    Brain.Screen.setPenColor(text_color);
    Brain.Screen.setFont(font);
    Brain.Screen.printAt(x_ + width_ / 2 - text_.length() * 5, y_ + height_ / 2 + 5,
                         text_.c_str());
}

Button::Button(const int& x, const int& y, const int& width, const int& height,
               const std::string& text, const std::function<void()>& callback)
    : Label(x, y, width, height, text), callback_(callback) {}
void Button::render(const color& text_color, const color& border_color,
                    const color& fill_color, const fontType font) {
    current_fill_color_ = fill_color;
    Brain.Screen.setPenColor(border_color);
    Brain.Screen.setFillColor(fill_color);
    Brain.Screen.drawRectangle(x_, y_, width_, height_, fill_color);
    Brain.Screen.setPenColor(text_color);
    Brain.Screen.setFont(font);
    Brain.Screen.printAt(x_ + width_ / 2 - text_.length() * 5, y_ + height_ / 2 + 5,
                         text_.c_str());
}

bool Button::in() {
    int x = Brain.Screen.xPosition();
    int y = Brain.Screen.yPosition();
    return x > x_ && x < x_ + width_ && y > y_ && y < y_ + height_;
}

void Button::check() {
    // Global press detection.
    static color original_color;
    if (!prev_overall_press_ && Brain.Screen.pressing() && in()) {
        original_color = getCurrentFillColor();
        render(black, black, color(128, 128, 128));
        detecting_ = true;
    }

    if (detecting_) {
        if (prev_in_ && !in()) {
            render(black, black, original_color);
        }
        if (!prev_in_ && in()) {
            render(black, black, color(128, 128, 128));
        }

        if (prev_overall_press_ && !Brain.Screen.pressing()) {
            if (in()) {
                callback_();
            }
            detecting_ = false;
        }
    }

    prev_in_ = in();
    prev_overall_press_ = Brain.Screen.pressing();
}
