#pragma once
#include "mathlib.h"

struct Sphere {
    Vector3 center;
    float radius;
};

struct Ray {
    Vector3 origin;
    Vector3 direction;
};