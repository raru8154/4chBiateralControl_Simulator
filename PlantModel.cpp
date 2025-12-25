# include "PlantModel.h"

PlantModel::PlantModel(double Mn, double Ktn, double dt)
    : Mn_(Mn), Ktn_(Ktn), dt_(dt) {}

double PlantModel::calculate(double Ia_ref, double F_env) {
    double X_ddot = (Ktn_*Ia_ref - F_env) / Mn_;

    X_dot_ += X_ddot*dt_;
    X_ += X_dot_*dt_;

    return X_;
}