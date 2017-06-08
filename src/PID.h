#ifndef PID_H
#define PID_H

#include <chrono>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;
  double target_speed_;

  double best_Kp_;
  double best_Ki_;
  double best_Kd_;

  double prev_cte_;

  double err_;
  double best_err_;

  chrono::high_resolution_clock::time_point time_;
  chrono::high_resolution_clock::time_point prev_time_;
  chrono::high_resolution_clock::time_point orig_time_;

  bool twiddle_is_init_;

  double *p_[3];
  double dp_[3];

  int which_K_;
  int up_or_down_;

  double speed_;
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the steering angle.
  */
  double Steer();

  /*
  * Calculate the throttle.
  */
  double Throttle();

  void Twiddle();
};

#endif /* PID_H */
