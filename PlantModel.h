# ifndef PLANT_MODEL_H
# define PLANT_MODEL_H

class PlantModel {
    public:
        PlantModel(double Mn, double Ktn, double dt);

        // 電流参照値と外乱から位置を計算
        double calculate(double Ia_ref, double F_env);
        double getPosition() const { return X_; }
        double getSpeed() const { return X_dot_; }
    
    private:
        const double Mn_;
        const double Ktn_;
        const double dt_;

        double X_ = 0.0;
        double X_dot_ = 0.0;
};

# endif // PLANT_MODEL_H