#ifndef MOTOR_SPEED_H_
#define MOTOR_SPEED_H_

#include "Arduino.h"
#include "macros.h"
#include "LowPassFilter.h"

//#define TIME_OVER(target,time) ((unsigned long)((time) - (target)) < 0x8000000U)

class MotorSpeed {

public:
    const unsigned short DELTA_MICROS_MIN  = 50;
    const unsigned short DELTA_MICROS_MAX  = 2000;
    const unsigned short INIT_PERIOD_MICROS_MIN = 10000;

    MotorSpeed(int sensor_pin, int timeout_ms, float filter_beta);

    //void init();
    void spin_once();
    bool isMotorRunning();

    float getSpeedRaw();
    float getSpeed();

    

private:
    static MotorSpeed *pMotorSpeed;
    LowPassFilter *lpf;

    bool is_motor_running_;
    bool is_measurement_valid_;
    short state_prev_;
    int sensor_pin_;
    int timeout_ms_;

    unsigned long pulse_start_us_;
    unsigned long period_us_;
    long period_delta_us_;

    unsigned long time_ms_;
    unsigned long timeout_target_ms_;

    //static void MotorSpeedIsr();
    void interruptHandler();
    static void interruptHandlerStatic();
    

};

#endif //MOTOR_SPEED_H_