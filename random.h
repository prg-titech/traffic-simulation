#ifndef RANDOM_H
#define RANDOM_H

#include <stdint.h>

uint32_t rand32(uint32_t* state);
const uint32_t RAND32_MAX = UINT32_MAX;

#endif  // RANDOM_H
