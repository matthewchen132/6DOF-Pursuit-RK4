#pragma once
#include <random>

class SimConfig {
public:
    uint32_t random_seed = 10;   // fixed default, overridable per run
    bool use_random_seed = false;
};

std::mt19937 make_rng(const SimConfig& cfg) {
    uint32_t seed = cfg.use_random_seed ? std::random_device{}() : cfg.random_seed;
    return std::mt19937(seed);
}