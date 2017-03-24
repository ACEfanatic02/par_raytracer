#pragma once

#ifdef _MPI
#include <mpi.h>
#else
#include <ctime>
#endif

struct Timer {
    #ifdef _MPI
    double start;
    #else
    clock_t start;
    #endif
};

inline void
Timer_Start(Timer * timer) {
    #ifdef _MPI
    timer->start = MPI_Wtime();
    #else
    timer->start = clock();
    #endif
}

inline double
Timer_Stop(Timer * timer) {
    #ifdef _MPI
    return MPI_Wtime() - timer->start;
    #else
    clock_t diff = clock() - timer->start;
    return (double)diff / (double)CLOCKS_PER_SEC;
    #endif
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
        fprintf(stderr, "[%s] :: %.2f s\n", _label, elapsed);
    }
};

#define TIME_BLOCK(l) ScopeTimer _timer_##__LINE__(l)
