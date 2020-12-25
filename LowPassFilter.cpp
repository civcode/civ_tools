#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float beta) : beta_(beta) {};

float LowPassFilter::filter(float raw) {
    if (val_ == 0.0f) {
        val_ = raw;
    } else {
        val_ = (1.0f - beta_) * val_ + beta_ * raw;
    }
    return val_;
}

float LowPassFilter::get_val() {
    return val_;
}

void LowPassFilter::reset() {
    val_ = 0.0f;
}

void LowPassFilter::setBeta(float beta) {
	beta_ = beta;
}



