#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

//struct for storing best params
struct params{
  std::vector<double> p; 
  std::vector<double> dp;
};

class Twiddle {
 public:
  /**
   * Constructor
   */
  Twiddle();

  /**
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Run one timestep of the twiddle method.
   */
  void Update();

  std::vector<double> p, dp;
  int num_params, idx;
  double best_err, avg_err, cur_err;
  bool flag_fw, is_initialized;
  params best_params;
};

#endif  // TWIDDLE_H