#pragma once
#include "brt.h"

struct RandomState {
    u64 _state[16];
    s32 _p;
};

void 
Random_Seed(RandomState * state, u64 seed) {
    if (seed == 0) {
        // xorshift cannot be seeded to 0
        seed = 0x5555555555555555ULL;
    }

    state->_p = 0;

    // We use an xorshift64* PRNG to seed the full state space
    // from this 64-bit seed.
    u64 x = seed;
    for (u32 i = 0; i < array_count(state->_state); ++i) {
        x ^= x >> 12;
        x ^= x >> 25;
        x ^= x >> 27;
        state->_state[i] = x * 2685821657736338717ULL;
    }
}

u64
Random_Next(RandomState * state) {
    u64 state_0 = state->_state[state->_p];
    state->_p++;
    state->_p &= 15;
    u64 state_1 = state->_state[state->_p];

    state_1 ^= state_1 << 31;
    state_1 ^= state_1 >> 11;
    state_0 &= state_0 >> 30;

    state->_state[state->_p] = state_0 ^ state_1;
    return state->_state[state->_p] * 1181783497276652981ULL;
}

inline s64
Random_NextInt(RandomState * state) {
    return (s64)Random_Next(state);
}

inline float
Random_NextFloat01(RandomState * state) {
    const u64 MAX_RAND_VALUE = 0xFFFFFFFFFFFFFFFFULL;
    // NOTE(bryan):  We're throwing away ~40 bits of random state here due to 
    // the precision difference, but I don't think it really matters?
    float f = (float)(Random_Next(state)) / (float)MAX_RAND_VALUE;
    return Clamp(f, 0.0f, 1.0f);
}

inline float
Random_NextFloat11(RandomState * state) {
    return (Random_NextFloat01(state) * 2.0f) - 1.0f;
}

inline bool
Random_WeightedChoice(RandomState * state, float probability) {
    return Random_NextFloat01(state) <= probability;
}

struct GaussianGenerator {
    RandomState * rng;
    bool generate;
    float _z;
};

float
Random_SampleGaussian(GaussianGenerator * gen, float mean, float stddev) {
    // Each pass generates two samples, so makes sure we use both.
    gen->generate = !gen->generate;
    if (!gen->generate) {
        return gen->_z * stddev + mean;
    }

    float u;
    float v;
    float d;
    do {
        u = Random_NextFloat11(gen->rng);
        v = Random_NextFloat11(gen->rng);
        d = u*u + v*v;
    } while (d == 0.0f || d >= 1.0f);

    float log_factor = sqrtf(-2.0f * logf(d));
    gen->_z = v * log_factor;
    return u * log_factor * stddev + mean;
}