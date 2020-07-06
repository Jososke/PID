#include "twiddle.h"
#include <cmath>
#include <iostream>

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double kp_, double ki_, double kd_) {
  best_params.p = {0, 0, 0};
  best_params.dp = {0, 0, 0};
  //setting the initial pid values
  p = {kp_, ki_, kd_};
  //setting the dp values - integral gain should be low so I am biasing that value
  dp = {1.0, 0.01, 1.0};
}

void Twiddle::Update() {

  // initialize
  if (!is_initialized) {
    best_err = cur_err;
    flag_fw = true;

    p[idx] += dp[idx];
    is_initialized = true;
  }
  else {
    if (cur_err < best_err) {
      best_err = cur_err;
      best_params.p = p;
      best_params.dp = dp;
      dp[idx] *= 1.1;
      idx = (idx + 1) % 3;
      flag_fw = true;
    }
    else {
      if (flag_fw) {
        p[idx] -= 2 * dp[idx];
        flag_fw = false;
      }
      else {
        p[idx] += dp[idx];
        dp[idx] *= 0.9;
        idx = (idx + 1) % 3;
        flag_fw = true;
      }
    }
    if (flag_fw) {
      p[idx] += dp[idx];
    }
  }
}