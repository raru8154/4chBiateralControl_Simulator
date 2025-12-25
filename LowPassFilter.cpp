# include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double g, double dt)
    : g_(g), dt_(dt) {}

double LowPassFilter::calculate(double input) {
    double y = integral_;
    double error_input = input - y;
    integral_ += g_ * error_input * dt_;
    double y_next = integral_;
    return y_next;
}