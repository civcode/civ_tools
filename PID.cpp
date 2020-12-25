#include "PID.h"
#include "Arduino.h"

PID::PID(float Kp, float Ki, float Kd, float u_min, float u_max) : 
    Kp_(Kp),
    Ki_(Ki),
    Kd_(Kd),
    u_min_(u_min),
    u_max_(u_max),
    delta_t_(0.0f),
    integral_(0.0f),
    e_prev_(0.0f),
    time_us_prev_(0),
    is_first_iteration_(true) 
{};

void PID::resetIntegral() {
    integral_ = 0.0f;
}

void PID::setGainParameters(float Kp, float Ki, float Kd) {
	Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

float PID::calculate(float setpoint, float process_value, unsigned long time_us) {

    if (is_first_iteration_) {
        //Serial.println("first iteration");
        time_us_prev_ = time_us_prev_;
        is_first_iteration_ = false;
        return 0.0f;
    }

    //Serial.println("calculate");

    float dt = static_cast<float>(time_us - time_us_prev_) / 1E6f;
    time_us_prev_ = time_us;

    //Serial.println(dt);

    float e = setpoint - process_value;

    //Serial.println(e);

    integral_ += e * dt;

    //Serial.println(integral_);
    
    float derivative = (e - e_prev_) / dt;
    e_prev_ = e;

    float u = Kp_ * e + Ki_ * integral_ + Kd_ * derivative;

    //Serial.println(u);

    if (u > u_max_) {
        u = u_max_;
    } else if (u < u_min_) {
        u = u_min_;
    }

    return u;
}