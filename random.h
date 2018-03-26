#ifndef RANDOM_H
#define RANDOM_H

#include <stdint.h>

#if defined(__CUDA_ARCH__)
#define HOST_DEVICE __host__ __device__
#else
#define HOST_DEVICE
#endif

HOST_DEVICE inline uint32_t rand32(uint32_t* state)
{
  return *state = ((uint64_t)*state * 279470273u) % 0xfffffffb;
}

const uint32_t RAND32_MAX = UINT32_MAX;

#endif  // RANDOM_H
