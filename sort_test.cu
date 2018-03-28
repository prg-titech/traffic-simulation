/******************************************************************************
 * Copyright (c) 2011, Duane Merrill.  All rights reserved.
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Simple example of DeviceRadixSort::SortPairs().
 *
 * Sorts an array of float keys paired with a corresponding array of int values.
 *
 * To compile using the command line:
 *   nvcc -arch=sm_XX example_device_radix_sort.cu -I../.. -lcudart -O3
 *
 ******************************************************************************/

// Ensure printing of CUDA runtime errors to console
#define CUB_STDERR

#include <stdio.h>
#include <algorithm>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

#include "cub-1.8.0/cub/util_allocator.cuh"
#include "cub-1.8.0/cub/device/device_radix_sort.cuh"

using namespace cub;
using namespace std;

int main() {
  // Declare, allocate, and initialize device-accessible pointers for sorting data
  int h_arr_keys[] = {1, 3, 6, 1, 8, 4, 19, 0, 0, 1, 45, 2, 4, 2, 2, 9};
  int h_arr_vals[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  int  num_items = 16;          // e.g., 7
  int  *d_keys_in;         // e.g., [8, 6, 7, 5, 3, 0, 9]
  int  *d_keys_out;        // e.g., [        ...        ]
  int  *d_values_in;       // e.g., [0, 1, 2, 3, 4, 5, 6]
  int  *d_values_out;      // e.g., [        ...        ]
  
  cudaMalloc((void**) &d_keys_in, 4*16);
  cudaMalloc((void**) &d_keys_out, 4*16);
  cudaMalloc((void**) &d_values_in, 4*16);
  cudaMalloc((void**) &d_values_out, 4*16);
  gpuErrchk(cudaDeviceSynchronize());

  cudaMemcpy(d_keys_in, h_arr_keys, 4*16, cudaMemcpyHostToDevice);
  cudaMemcpy(d_values_in, h_arr_vals, 4*16, cudaMemcpyHostToDevice);
  gpuErrchk(cudaDeviceSynchronize());

  // Determine temporary device storage requirements
  void     *d_temp_storage = NULL;
  size_t   temp_storage_bytes = 0;
  cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
      d_keys_in, d_keys_out, d_values_in, d_values_out, num_items);
  // Allocate temporary storage
  cudaMalloc(&d_temp_storage, temp_storage_bytes);
  // Run sorting operation
  cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
      d_keys_in, d_keys_out, d_values_in, d_values_out, num_items);
  // d_keys_out            <-- [0, 3, 5, 6, 7, 8, 9]
  // d_values_out          <-- [5, 4, 3, 1, 2, 0, 6]
  gpuErrchk(cudaDeviceSynchronize());

  cudaMemcpy(h_arr_keys, d_keys_out, 4*16, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_arr_vals, d_values_out, 4*16, cudaMemcpyDeviceToHost);
  gpuErrchk(cudaDeviceSynchronize());

  for(int i = 0; i < 16; ++i) {
    printf("%i         %i\n", h_arr_keys[i], h_arr_vals[i]);
  }

}
