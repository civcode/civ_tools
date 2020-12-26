
#include "SlewRate.h"

#include <cmath>

SlewRate::SlewRate(float slew_rate, float v_min, float v_max) : 
    slew_rate_(slew_rate), 
    v_min_(v_min), 
    v_max_(v_max) 
{
        reset();
};

float SlewRate::filter(float u) {
    float du = u - u_prev_;
    float u_out;
    if (fabs(du) > slew_rate_) {
        if (du > 0) {
            u_out = u_prev_ + slew_rate_;
        } else {
            u_out = u_prev_ - slew_rate_;
        }
    } else {
        u_out = u;   
    }
    u_out = saturation(u_out);
    u_prev_ = u_out;
    return u_out;
}

void SlewRate::reset() {
    u_prev_ = 0.0f;
}

void SlewRate::setSlewRate(float slew_rate) {
    slew_rate_ = slew_rate;
}

void SlewRate::setSaturation(float v_min, float v_max) {
    v_min_ = v_min;
    v_max_ = v_max;
}

float SlewRate::saturation(float u) {
    if (u > v_max_) {
        u = v_max_;
    } else if (u < v_min_) {
        u = v_min_;
    }
    return u;
}