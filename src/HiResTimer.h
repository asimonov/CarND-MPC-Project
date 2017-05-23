//
// Created by Alexey Simonov on 23/05/2017.
//

#ifndef MPC_HIRESTIMER_H
#define MPC_HIRESTIMER_H

#include <chrono>

using namespace std;
using namespace std::chrono;

class HiResTimer {
public:
    HiResTimer();
    void Reset();
    double GetElapsedSecs();
private:
    high_resolution_clock::time_point start_time_point_;
};


#endif //MPC_HIRESTIMER_H
