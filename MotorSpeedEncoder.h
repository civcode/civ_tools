#ifndef MOTOR_SPEED_ENCODER_H_
#define MOTOR_SPEED_ENCODER_H_

#include "Arduino.h"
#include "macros.h"
#include "LowPassFilter.h"

//#define TIME_OVER(target,time) ((unsigned long)((time) - (target)) < 0x80000000U)

class MotorSpeedEncoder {

public:
    const unsigned short DELTA_MICROS_MIN  = 1;//50;
    const unsigned short DELTA_MICROS_MAX  = 5000;//2000;
    const unsigned short INIT_PERIOD_MICROS_MIN = 1000;

    const float ENCODER_PPR = 341.2f;

    //MotorSpeedEncoder(int sensor_pin, int timeout_ms, float filter_beta);
    MotorSpeedEncoder(int sensor_pin_c1, int sensor_pin_c2, int timeout_ms, float filter_beta);

    //void init();
    void spin_once();
    bool isMotorRunning();

    float getSpeedRaw();
    float getSpeed();

    int getEncoderCount();
	
	void setFilterBeta(float filter_beta);

    

private:
    static MotorSpeedEncoder *pMotorSpeedEncoder;
    LowPassFilter *lpf;

    bool is_motor_running_;
    bool is_measurement_valid_;
    short state_prev_;
    int sensor_pin_c1_;
    int sensor_pin_c2_;
    int timeout_ms_;

    int encoder_count_;

    unsigned long pulse_start_us_;
    unsigned long period_us_;
    long period_delta_us_;

    unsigned long time_ms_;
    unsigned long timeout_target_ms_;

    //static void MotorSpeedIsr();
    void interruptHandler();
    static void interruptHandlerStatic();
    

};

#endif //MOTOR_SPEED_ENCODER_H_
