#ifndef POWERLIMIT_H
#define POWERLIMIT_H

#include <stdint.h>

#include "Pid.h"

// Use GCC/ARMCC/Clang compatible packed struct definition
struct model_t {
    double k1;
    double k2;
    double k3;
    double a;
    double real_current;
    float PowLim_IQ;
} __attribute__((packed, aligned(1)));
typedef struct model_t model_t;

struct cap_t {
    double k1;
    double k2;
    double k3;
    double k4;
    double a;
    double V;
} __attribute__((packed, aligned(1)));
typedef struct cap_t cap_t;

struct power_t {
    float motor;
    float cap;
    float total;
} __attribute__((packed, aligned(1)));
typedef struct power_t power_t;

extern model_t model[4];
extern power_t power;

void PowertotalLimit(void);

#endif
