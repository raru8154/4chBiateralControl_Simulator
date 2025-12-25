# ifndef DISTURBANCE_OBSERVER_H
# define DISTURBANCE_OBSERVER_H

# include "LowPassFilter.h"

class DisturbanceObserver {
    public: 
        DisturbanceObserver(double Mn, double Ktn, double dt, double gd);
    
        // メソッド
        double calculate(double Ia_ref, double X_dot);

    private:
        const double Mn_;
        const double Ktn_;
        const double dt_;
        const double gd_;

        LowPassFilter lpf_;
};

# endif // DISTURBANCE_OBSERVER_H