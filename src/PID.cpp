#include "PID.h"
#include <cmath>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients and errors
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  steps = 0;
  total_err = 0;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error; //p_error is cte from one timestep before
  p_error = cte;
  i_error += cte;
  steps++;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp * p_error - Ki * i_error - Kd * d_error;;
}

double PID::SquareError() {
  total_err += pow(p_error,2);
	return total_err;
}