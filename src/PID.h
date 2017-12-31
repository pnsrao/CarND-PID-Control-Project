#include <ctime>
#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  * Calculate the total PID error.
  */
  double TotalError();

  /* Hold time data structure 
   */
  std::clock_t prev_time;
  /*
   * Additional function and variables for twiddle
   */
  bool twiddle();
  bool inTwiddle;
  double dp[3]; //for P,I and D parameters
  double p[3];
  double cum_err; //Tracks error across time steps
  double best_err; // Maintains the best error for the twiddle algorithm
  int twiddle_index;
  int twiddle_dir;
  int iStep;
  void pid_resetmeas();
};

#endif /* PID_H */
