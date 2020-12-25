#ifndef PID_H_
#define PID_H_

class PID {
public:
    PID(float Kp, float Ki, float Kd, float u_min, float u_max);
    float calculate(float setpoint, float process_value, unsigned long time_us);
    void resetIntegral();
	void setGainParameters(float Kp, float Ki, float Kd);

private:
    float Kp_;
    float Ki_;
    float Kd_;
    float u_min_;
    float u_max_;
    float delta_t_;
    float integral_;
    float e_prev_;

    unsigned long time_us_prev_;

    bool is_first_iteration_;
};

#endif //PID_CONTROLLER_H_