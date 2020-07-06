#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <float.h> //DBL_MAX

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
  params best_params;

  bool flag_fw = false;
  double best_err = DBL_MAX;
  double cur_err = 0;
  double avg_err = 0;
  int idx = 0;
  bool is_initialized = false;
};

#endif  // TWIDDLE_H