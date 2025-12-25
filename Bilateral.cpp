// 実行コマンドは以下
// g++ Bilateral.cpp PlantModel.cpp DisturbanceObserver.cpp PseudoDifferential.cpp LowPassFilter.cpp -o bilateral_sim -lm && ./bilateral_sim

// datファイルの見方はこう
//"Time, Xl, Xf, Xl_dot, Xf_dot, Fl_dis, Ff_dis, Fl_hat_dis, Ff_hat_dis, Ia_l_ref, Ia_f_ref, Xl_human\n"

# include <stdio.h>
# include <math.h>

# include "LowPassFilter.h"
# include "PseudoDifferential.h"
# include "PlantModel.h"
# include "DisturbanceObserver.h"

using namespace std;

const double Mn = 0.55;
const double Ktn = 6.0;
//const double Kp = 1600.0; //比例ゲイン
const double Kp = 10000.0 ; //比例ゲイン
const double Kd = 80.0; //微分ゲイン
const double Kf = 1.0; //力ゲイン
const double gd = 500.0; //カットオフ周波数

const double A = 0.1; //人間の動きの振幅
const double f = 2.0; //人間の動きの周波数[Hz]

const double k_human = 500.0; //人間の剛性
const double c_human = 50.0; //人間の減衰定数
const double k_env = 10000.0; //環境の剛性
const double c_env = 100.0; //環境の減衰定数

const double X_env = 0.1; //環境の位置

// 等倍スケーリング
//const double scale_alpha = 1.0;
//const double scale_beta = 1.0;

// α=2.0, β=2.0にスケーリング
const double scale_alpha = 1.0;
const double scale_beta = 2.0;

const double dt = 0.001; //時間刻み
const double t_end = 1.0;

void calculate_current(double Xl, double Xf, double Ia_l_ref, double Ia_f_ref,
                       double Xl_dot, double Xf_dot,
                       DisturbanceObserver& dob_leader, DisturbanceObserver& dob_follower,
                       PseudoDifferential& pseudo_diff_D,
                       double& Ia_l_ref_out, double& Ia_f_ref_out,
                       double& Fl_hat_dis, double& Ff_hat_dis) {
    
    // PD制御
    double position_error = scale_alpha*Xf - Xl;
    double pseudo_diff_output = pseudo_diff_D.calculate(position_error);
    double PD_signal = scale_beta*(Kp*position_error + Kd*pseudo_diff_output);

    // DOBで外乱推定
    Fl_hat_dis = dob_leader.calculate(Ia_l_ref, Xl_dot);
    Ff_hat_dis = dob_follower.calculate(Ia_f_ref, Xf_dot);

    // 力を計算
    double F_signal = scale_alpha*Kf*(-Ff_hat_dis - scale_beta*Fl_hat_dis);

    // 加速度参照値計算
    double Xl_ddot_ref_out = PD_signal + F_signal;
    double Xf_ddot_ref_out = -PD_signal + F_signal;

    // 電流参照値
    Ia_l_ref_out = Mn*Xl_ddot_ref_out/Ktn + Fl_hat_dis/Ktn;
    Ia_f_ref_out = Mn*Xf_ddot_ref_out/Ktn + Ff_hat_dis/Ktn;
}

int main() {
    // インスタンス生成
    PlantModel plant_leader(Mn, Ktn, dt);
    PlantModel plant_follower(Mn, Ktn, dt);
    DisturbanceObserver dob_leader(Mn, Ktn, dt, gd);
    DisturbanceObserver dob_follower(Mn, Ktn, dt, gd);
    PseudoDifferential pseudo_diff_leader(gd, dt);
    PseudoDifferential pseudo_diff_follower(gd, dt);
    PseudoDifferential pseudo_diff_D(gd, dt);

    // 時間
    //double t = 0.0;

    // ロボットの位置と速度
    double Xl = 0.0;
    double Xl_dot = 0.0;
    double Xf = 0.0;
    double Xf_dot = 0.0;

    // 人間の位置
    double Xl_human = 0.0;
    double Xl_human_dot = 0.0;
    
    // 電流参照値
    double Ia_l_ref = 0.0;
    double Ia_f_ref = 0.0;
    double Ia_l_ref_prev = 0.0;
    double Ia_f_ref_prev = 0.0;

    // 外乱と推定外乱
    double Fl_dis = 0.0;
    double Ff_dis = 0.0;
    double Fl_hat_dis = 0.0;
    double Ff_hat_dis = 0.0;

    // ファイルオープン
    FILE *fp = fopen("data_bilateral.dat", "w");
    if (fp == NULL) {
        printf("cannot open\n");
        return 1;
    }

    // シミュレーションループ
    for (double t = 0.0; t <= t_end; t += dt) {
        // 人間の動きの更新
        Xl_human = A * (1.0 - cos(2.0 * M_PI * f * t));
        Xl_human_dot = A * 2.0 * M_PI * f * sin(2.0 * M_PI * f * t);
        
        // ロボットの状態を取得（シミュレーションなのでプラントから）
        Xl = plant_leader.getPosition();
        Xf = plant_follower.getPosition();
        Xl_dot = pseudo_diff_leader.calculate(Xl);
        Xf_dot = pseudo_diff_follower.calculate(Xf);

        // 外乱の計算
        Fl_dis = k_human*(Xl - Xl_human) + c_human*(Xl_dot - Xl_human_dot);

        // 自由応答
        //Ff_dis = 0.0;
        
        if (Xf > X_env){
            Ff_dis = k_env*(Xf - X_env) + c_env*Xf_dot;
        }else{
            Ff_dis = 0.0; //接触していないとき反力は0
        }

        // 電流参照値と推定外乱の計算・更新
        Ia_l_ref_prev = Ia_l_ref;
        Ia_f_ref_prev = Ia_f_ref;

        calculate_current(Xl, Xf, Ia_l_ref_prev, Ia_f_ref_prev,
                          Xl_dot, Xf_dot,
                          dob_leader, dob_follower,
                          pseudo_diff_D,
                          Ia_l_ref, Ia_f_ref,
                          Fl_hat_dis, Ff_hat_dis);
        
        // プラントの更新
        plant_leader.calculate(Ia_l_ref, Fl_dis);
        plant_follower.calculate(Ia_f_ref, Ff_dis);

        // データ記録
        fprintf(fp, "%.3f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                t, Xl, Xf, Xl_dot, Xf_dot,
                -Fl_dis, Ff_dis,
                Fl_hat_dis, Ff_hat_dis,
                Ia_l_ref, Ia_f_ref, Xl_human);
        fflush(fp);
    }
    fclose(fp);
    return 0;
}