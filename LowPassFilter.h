#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

class LowPassFilter {
public:
    LowPassFilter(float beta);
    float filter(float raw);
    float get_val();
    void reset();
	void setBeta(float beta);

private:
    float beta_;
    float val_;
};

#endif //LOW_PASS_FILTER_H_