#include "MotorSpeedEncoder.h"
//
//MotorSpeed *pMotorSpeed;

MotorSpeedEncoder *MotorSpeedEncoder::pMotorSpeedEncoder = nullptr; 

/*
void interruptHandlerImplExt() {
    Serial.println("interrupt impl");
    pMotorSpeed->interruptHandler();
}
*/

MotorSpeedEncoder::MotorSpeedEncoder(int sensor_pin, int timeout_ms, float filter_beta) {
    sensor_pin_ = sensor_pin;
    timeout_ms_ = timeout_ms;

    is_motor_running_ = false;
    is_measurement_valid_ = false;
    state_prev_ = 0;
    pulse_start_us_ = 0;
    period_us_ = 0;
    period_delta_us_ = 0;
    time_ms_ = 0;
    timeout_target_ms_ = 0;

    encoder_count_ = 0;

    pMotorSpeedEncoder = this;

    lpf = new LowPassFilter(filter_beta);

    pinMode(sensor_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sensor_pin_), interruptHandlerStatic, CHANGE);
}


// void MotorSpeed::init() {
//     //instance = this;
//     //pinMode(sensor_pin_, INPUT_PULLUP);
//     //attachInterrupt(digitalPinToInterrupt(sensor_pin_), interruptHandlerImpl, CHANGE);
// }

void MotorSpeedEncoder::interruptHandlerStatic() {
    //Serial.println("interrupt impl");
    //pMotorSpeedEncoder->interruptHandler();
    pMotorSpeedEncoder->interruptHandler();
}

void MotorSpeedEncoder::interruptHandler() {

    //Serial.println("interrupt");
    timeout_target_ms_ = millis() + timeout_ms_;

    if (digitalRead(sensor_pin_) == HIGH) {
        //Serial.println("interrupt");
        if (state_prev_ == LOW) {
            if (is_measurement_valid_) {
                unsigned long cycle_us = micros() - pulse_start_us_;
                //delta_us_ = abs(static_cast<long>(period_us_) - (micros() - pulse_start_us_));
                period_delta_us_ = abs(static_cast<long>(period_us_) - cycle_us);
                //period_us_ = micros() - pulse_start_us_;
                
                //if ((is_motor_running_ && period_delta_us_ < DELTA_MICROS_MAX) || cycle_us > INIT_PERIOD_MICROS_MIN) {
                //if (((is_motor_running_ && period_delta_us_ < DELTA_MICROS_MAX) ||
                //    (is_motor_running_ && period_delta_us_ > DELTA_MICROS_MIN)) || cycle_us > INIT_PERIOD_MICROS_MIN) {
                //if ((is_motor_running_ && period_delta_us_ > 100) || cycle_us > INIT_PERIOD_MICROS_MIN) {
                if (is_motor_running_ || cycle_us > INIT_PERIOD_MICROS_MIN) {
                    is_motor_running_ = true;
                    period_us_ = cycle_us;
                    lpf->filter(static_cast<float>(period_us_));
                    
                } else {
                    period_us_ = 0;
                    lpf->filter(0.0f);
                }
                encoder_count_++;

            }
            pulse_start_us_ = micros();
            state_prev_ = HIGH;
        }
    } else {
        if (state_prev_ == HIGH) {
            state_prev_ = LOW;
            is_measurement_valid_ = true;
        }
    }
}

float MotorSpeedEncoder::getSpeedRaw() {
    //Serial.println("getSpeedRaw");
    if (period_us_ == 0) {
        return 0.0f;
    } else {
        return (1E6f / (ENCODER_PPR * period_us_));
    } 
}

float MotorSpeedEncoder::getSpeed() {
    // if (!is_motor_running_) {
    //     lpf->reset();
    // }

    if (lpf->get_val() == 0) {
        return 0.0f;
    } else {
        return (1E6f / (ENCODER_PPR * lpf->get_val()));
    }

}

int MotorSpeedEncoder::getEncoderCount() {
    return encoder_count_;
}

void MotorSpeedEncoder::setFilterBeta(float filter_beta) {
	lpf->setBeta(filter_beta);
}
		

bool MotorSpeedEncoder::isMotorRunning() {
    return is_motor_running_;
}

void MotorSpeedEncoder::spin_once() {
    time_ms_ = millis();
    if (TIME_OVER(timeout_target_ms_, time_ms_)) {
        period_us_ = 0;
        state_prev_ = LOW;
        is_motor_running_ = false;
        is_measurement_valid_ = false;
        lpf->reset();
        //timeout_target_ms_ = time_ms_ + timeout_ms_;
    }

}



/*
    const unsigned short DELTA_MICROS_MAX   = 1000;
    const unsigned short INIT_PERIOD_MICROS_MIN = 10000;

    bool is_motor_running_;
    bool is_measurement_valid_;
    byte state_prev_;

    unsigned long pulse_start_us_;
    unsigned long period_delta_us_;
    long delta_us_;
*/