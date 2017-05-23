//
// Created by Alexey Simonov on 23/05/2017.
//

#include "HiResTimer.h"

HiResTimer::HiResTimer() {
  Reset();
}

void HiResTimer::Reset() {
  start_time_point_ = high_resolution_clock::now();
}

double HiResTimer::GetElapsedSecs() {
  long long int mu = chrono::duration_cast<microseconds>(chrono::high_resolution_clock::now() - start_time_point_).count();
  return mu / 1e+6;
}
