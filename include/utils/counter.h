#ifndef COUNTER_H
#define COUNTER_H
#include "vex.h"
using namespace vex;

class Counter {
  private:
    int count_;
    int debounce_time_;
    bool prev_detected_;
    timer debounce_timer_;

  public:
    /**
     * @brief 构造函数，初始化计数器和去抖动时间(ms，默认为0)
     */
    Counter(bool detected, int debounce_time = 0);
    /**
     * @brief 设置去抖动时间
     * @param debounce_time 去抖动时间(ms)
     */
    void setDebounceTime(int debounce_time);
    /**
     * @brief 重置计数器，将球数设置为0
     * @param detected 当前检测状态
     */
    void reset(bool detected);
    /**
     * @brief 更新计数器
     * @param detected 当前检测状态
     */
    void update(bool detected);
    /**
     * @brief 获取当前计数值
     * @return 当前计数值
     */
    int getCount();
};

#endif // COUNTER_H