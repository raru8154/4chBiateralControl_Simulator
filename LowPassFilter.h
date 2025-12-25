# ifndef LOW_PASS_FILTER_H
# define LOW_PASS_FILTER_H

class LowPassFilter {
    public:
        LowPassFilter(double g, double dt);

        double calculate(double input);

    private:
        const double g_;
        const double dt_;

        double integral_ = 0.0;
};

# endif // LOW_PASS_FILTER_H