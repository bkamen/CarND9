#ifndef PID_H
#define PID_H
#include <vector>

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
  * Help variable for cte of last step
  */
  double last_cte;

  /*
  * Twiddle Parameters
  */

  double thresh;
  std::vector<double> dp; // vector with parameter deltas
  int p_idx; // parameter index
  int step; // current step in sim_period
  int sim_period; // amount of steps in a sim_period
  bool twiddle; // flag of twiddle should be enabled or not
  double error_sum; // sum of squared errors in current sim_period
  double error_min; // minimal error for optimisation check
  bool add_step; // flag if add step was tried
  bool sub_step; // flag if sub step was tried
  int n_runs; // number of simulation runs
  int n_settle; // number of steps to let the car settle

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
  void Init(double Kp, double Ki, double Kd, bool twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Change controller parameter
  */
  void AdjustParam(int idx, double delta);
};

#endif /* PID_H */
