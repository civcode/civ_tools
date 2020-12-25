#ifndef SLEW_RATE_H_
#define SLEW_RATE_H_

class SlewRate {

public:
    SlewRate(float slew_rate, float v_min, float v_max);
    float filter(float u);
    void reset();
    void setSlewRate(float slew_rate);
    void setSaturation(float v_min, float v_max);

private:
    float slew_rate_;
    float v_min_;
    float v_max_;
    float u_prev_;

    float saturation(float u);
};



#endif // SLEW_RATE_H_ 

