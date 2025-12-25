# ifndef PSEUDODIFFERENTIAL_H
# define PSEUDODIFFERENTIAL_H

# include "LowPassFilter.h"

class PseudoDifferential {
    public:
        PseudoDifferential(double g, double dt);

        double calculate(double input);

    private:
        LowPassFilter lpf_;
        const double dt_;

        double prev_lpf_output_ = 0.0;
};

# endif // PSEUDODIFFERENTIAL_H