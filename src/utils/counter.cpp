#include "utils/counter.h"
#include "robot-config.h"

#include "params.h"

Counter::Counter(bool detected, int debounce_time)
    : count_(0), debounce_time_(debounce_time), prev_detected_(detected) {}
void Counter::setDebounceTime(int debounce_time) { debounce_time_ = debounce_time; }
void Counter::reset(bool detected) {
    count_ = 0;
    prev_detected_ = detected;
    debounce_timer_.reset();
}
void Counter::update(bool detected) {
    if (debounce_timer_.time() >= debounce_time_) {
        if (prev_detected_ && !detected) {
            count_++;
        }
        prev_detected_ = detected;
        debounce_timer_.reset();
    }
}
int Counter::getCount() { return count_; }