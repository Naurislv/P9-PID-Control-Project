#include <iostream>
#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Kcte, double Ksteer, double Kspeed) {

    i_error = 0; // integral error
    is_initialized_ = false; // wether algorithm is initialized

    _p.resize(3); // coefficient for PID controller
    _p2.resize(3); // coefficient for PID controller

    _p[0] = Kp; // position coefficient
    _p[1] = Ki; // integral coefficient
    _p[2] = Kd; // differential coefficient

    _p2[0] = Kcte; // differential coefficient
    _p2[1] = Ksteer; // differential coefficient
    _p2[2] = Kspeed; // differential coefficient

    _dp.resize(3); // koefficients for PID controller
    _dp2.resize(3); // koefficients for PID controller

    _dp[0] = 0.0035; // position coefficient
    _dp[1] = 0.00035; // integral coefficient
    _dp[2] = 0.025; // differential coefficient

    _dp2[0] = 0.084; // differential coefficient
    _dp2[1] = 0.11; // differential coefficient
    _dp2[2] = 0.139; // differential coefficient

    start_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
}

void PID::UpdateError(double cte, double speed_value, double angle_value) {
    if (!is_initialized_) {
        p_error = cte;
        is_initialized_ = true;
    }

    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;

    speed_error = fabs(speed_value);
    steer_error = fabs(angle_value);

    milliseconds current_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    dt = current_time.count() - start_time.count();

    speed_sum += speed_error * (1 / (fabs(cte) + 0.1)) / 100;
    cte_sum += cte * cte;
}

double PID::Predict_steer(){
    return - _p[0] * p_error - _p[1] * i_error - _p[2] * d_error;
}

double PID::Predict_throttle(){
    return _p2[0] * (1 / (fabs(p_error) + 0.0001)) +
           _p2[1] * (1 / (steer_error + 0.0001)) +
           _p2[2] * (1 / (speed_error + 0.0001));
}

void PID::Twiddle(int& step, int& i, double& err, double& best_err,
                  std::vector<double>& p, std::vector<double>& dp){
    std::cout << "\nTWIDDLING" << '\n';

    if (step == 0) {
        p[i] += dp[i];
        step = 1;
    } else if (step == 1){
        if (err < best_err) {
            best_err = err;
            dp[i] *= 1.1;

            i += 1;
            if (i == p.size()) {
                i = 0;
            }

            p[i] += dp[i];
        } else {
            p[i] -= 2 * dp[i];
            step = 2;
        }
    } else if (step == 2){
        if (err < best_err) {
            best_err = err;
            dp[i] *= 1.1;
        } else {
            p[i] += dp[i];
            dp[i] *= 0.9;
        }

        step = 1;
        i += 1;
        if (i == p.size()) {
            i = 0;
        }

        p[i] += dp[i];
    }

    start_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    speed_sum = 0;
    cte_sum = 0;
    std::cout << " K0 " << p[0] << " K1 " <<  p[1] << " K2 " << p[2]
              << "\n dp0 " << dp[0] << " dp1 " <<  dp[1] << " dp2 " << dp[2]
              << "\n err " << err << " --> Best_err " << best_err << "\n\n\n";
}
