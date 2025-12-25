# include "PseudoDifferential.h"

PseudoDifferential::PseudoDifferential(double g, double dt)
    : lpf_(g, dt), dt_(dt) {}

double PseudoDifferential::calculate(double input) {
    double lpf_output = lpf_.calculate(input);
    
    // 後進オイラー法で離散化
    double derivative_output = (lpf_output - prev_lpf_output_)/dt_;
    prev_lpf_output_ = lpf_output;
    return derivative_output;
}