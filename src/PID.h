#ifndef PID_H
#define PID_H

#include <vector>
#include <chrono>

using namespace std::chrono;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

   double speed_error;
   double steer_error;

  /*
  * Coefficients
  */
  bool is_initialized_;
  std::vector<double> _p;
  std::vector<double> _p2;

  /*
  * Twiddling coefficients
  */
  std::vector<double> _dp;
  std::vector<double> _dp2;
  double speed_sum = 0;
  double cte_sum = 0;
  long dt;
  milliseconds start_time;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double Kcte, double Ksteer, double Kspeed);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed_value, double angle_value);

  /*
  * Calculate new PID coefficients
  */
  void Twiddle(int& step, int& i, double& err, double& best_err,
               std::vector<double>& p, std::vector<double>& dp);

  /*
  * Calculate new PID value
  */
  double Predict_steer();
  double Predict_throttle();
};

#endif /* PID_H */
