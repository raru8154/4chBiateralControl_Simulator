# include "DisturbanceObserver.h"

DisturbanceObserver::DisturbanceObserver(double Mn, double Ktn, double dt, double gd)
    : Mn_(Mn), Ktn_(Ktn), dt_(dt), gd_(gd), lpf_(gd, dt) {}

double DisturbanceObserver::calculate(double Ia_ref, double X_dot) {
    double lpf_input = Ktn_*Ia_ref + Mn_*gd_*X_dot;
    double lpf_output = lpf_.calculate(lpf_input);
    double F_hat_env = lpf_output - Mn_*gd_*X_dot;
    return F_hat_env;
}