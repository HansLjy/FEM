//
// Created by hansljy on 11/1/22.
//

#ifndef FEM_TIMER_H
#define FEM_TIMER_H

#include <ctime>
#include "spdlog/spdlog.h"
#include <chrono>

#define START_TIMING(name) \
               auto name##_start_time = clock();  \
               int name##_total_time = 0;

#define PAUSE_TIMING(name) \
               name##_total_time += clock() - name##_start_time;

#define RESUME_TIMING(name) \
               name##_start_time = clock();

#define STOP_TIMING(description, name) \
               name##_total_time += clock() - name##_start_time; \
               spdlog::info("Total time on timer {}: {} ticks = {} ms", description, name##_total_time, 1000.0 * name##_total_time / CLOCKS_PER_SEC);

#endif //FEM_TIMER_H
