#pragma once

#include <ctime>

struct Timer {
    clock_t start;
};

inline void
Timer_Start(Timer * timer) {
    timer->start = clock();
}

inline double
Timer_Stop(Timer * timer) {
    clock_t diff = clock() - timer->start;
    return (double)diff / (double)CLOCKS_PER_SEC;
}

struct ScopeTimer {
    Timer _t;
    const char * _label;
    ScopeTimer(const char * l) : 
        _label(l) { 
        Timer_Start(&_t); 
    }
    
    ~ScopeTimer() {
        double elapsed = Timer_Stop(&_t);
        printf("[%s] :: %.2f s\n", _label, elapsed);
    }
};

#define TIME_BLOCK(l) ScopeTimer _timer_##__LINE__(l)
