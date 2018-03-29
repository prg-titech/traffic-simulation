#include <chrono>

#include "traffic_aos_int_cuda.h"
#include "random.h"

#include "cub-1.8.0/cub/util_allocator.cuh"
#include "cub-1.8.0/cub/device/device_radix_sort.cuh"

using namespace cub;

using namespace std;

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

namespace simulation {
namespace aos_int {
extern Cell* s_Cell;
extern IndexType s_size_Cell;

extern IndexType* s_outgoing_cells;
extern IndexType s_size_outgoing_cells;

extern IndexType* s_incoming_cells;
extern IndexType s_size_incoming_cells;

extern Car* s_Car;
extern IndexType s_size_Car;

extern IndexType* s_car_paths;
extern IndexType s_size_car_paths;

extern IndexType* s_inactive_cars;
extern IndexType s_size_inactive_cars;

extern TrafficLight* s_TrafficLight;
extern IndexType s_size_TrafficLight;

extern PriorityYieldTrafficController* s_PriorityYieldTrafficController;
extern IndexType s_size_PriorityYieldTrafficController;

extern SharedSignalGroup* s_SharedSignalGroup;
extern IndexType s_size_SharedSignalGroup;

extern IndexType* s_traffic_light_signal_groups;
extern IndexType s_size_traffic_light_signal_groups;

extern IndexType* s_priority_ctrl_signal_groups;
extern IndexType s_size_priority_ctrl_signal_groups;

extern IndexType* s_signal_group_cells;
extern IndexType s_size_signal_group_cells;

extern Simulation* instance;
}  // namespace aos_int

namespace aos_int_cuda {

// Singleton simulation instance.
__device__ Simulation* instance;

// Data storage.
__device__ Cell* s_Cell;
__device__ IndexType s_size_Cell = 0;

__device__ IndexType* s_outgoing_cells;
__device__ IndexType s_size_outgoing_cells = 0;

__device__ IndexType* s_incoming_cells;
__device__ IndexType s_size_incoming_cells = 0;

__device__ Car* s_Car;
__device__ Car* s_tmp_Car;  // temp array used for changing physical order
__device__ IndexType s_size_Car = 0;

__device__ IndexType* s_car_paths;
__device__ IndexType s_size_car_paths = 0;

__device__ IndexType* s_inactive_cars;
__device__ IndexType s_size_inactive_cars = 0;

__device__ TrafficLight* s_TrafficLight;
__device__ IndexType s_size_TrafficLight = 0;

__device__ PriorityYieldTrafficController* s_PriorityYieldTrafficController;
__device__ IndexType s_size_PriorityYieldTrafficController = 0;

__device__ SharedSignalGroup* s_SharedSignalGroup;
__device__ IndexType s_size_SharedSignalGroup = 0;

__device__ IndexType* s_traffic_light_signal_groups;
__device__ IndexType s_size_traffic_light_signal_groups = 0;

__device__ IndexType* s_priority_ctrl_signal_groups;
__device__ IndexType s_size_priority_ctrl_signal_groups = 0;

__device__ IndexType* s_signal_group_cells;
__device__ IndexType s_size_signal_group_cells = 0;

__device__ IndexType* s_Car_reorder_in;
__device__ IndexType* s_Car_reorder_keys_in;

__device__ IndexType* s_Car_reorder;
__device__ IndexType* s_Car_reorder_keys;

IndexType* d_car_reorder_in;
IndexType* d_car_reorder_keys_in;
IndexType* d_car_reorder;
IndexType* d_car_reorder_keys;

#define MEMCPY_TO_DEVICE(class, var) \
  class* dev_ ## var; \
  gpuErrchk(cudaMalloc((void**) &dev_ ## var, \
             sizeof(class)*simulation::aos_int::s_size_ ## var)); \
  printf("GPU allocation for " STRINGIFY(var) ": %i bytes, %i objects\n", \
         sizeof(class)*simulation::aos_int::s_size_ ## var, \
         simulation::aos_int::s_size_ ## var); \
  gpuErrchk(cudaMemcpyToSymbol(s_ ## var, \
                     &dev_ ## var, sizeof(char*))); \
  gpuErrchk(cudaMemcpy((void*) dev_ ## var, \
             (void*) simulation::aos_int::s_ ## var, \
             sizeof(class)*simulation::aos_int::s_size_ ## var, \
             cudaMemcpyHostToDevice)); \
  gpuErrchk(cudaMemcpyToSymbol(s_size_ ## var, \
                     &simulation::aos_int::s_size_ ## var, \
                     sizeof(IndexType)));
#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)

void initialize() {
  MEMCPY_TO_DEVICE(Cell, Cell);
  MEMCPY_TO_DEVICE(IndexType, outgoing_cells);
  MEMCPY_TO_DEVICE(IndexType, incoming_cells);
  MEMCPY_TO_DEVICE(Car, Car);
  MEMCPY_TO_DEVICE(IndexType, car_paths);
  MEMCPY_TO_DEVICE(TrafficLight, TrafficLight);
  MEMCPY_TO_DEVICE(PriorityYieldTrafficController,
                   PriorityYieldTrafficController);
  MEMCPY_TO_DEVICE(SharedSignalGroup, SharedSignalGroup);
  MEMCPY_TO_DEVICE(IndexType, traffic_light_signal_groups);
  MEMCPY_TO_DEVICE(IndexType, priority_ctrl_signal_groups);
  MEMCPY_TO_DEVICE(IndexType, signal_group_cells);

  for (int i = 0; i < simulation::aos_int::s_size_Car; ++i) {
    // Fix path_: Contains a pointer.
    auto& car_path = ((Car*) simulation::aos_int::s_Car)[i].path();
    auto path_offset = car_path.buffer()
        - simulation::aos_int::s_car_paths;
    fixed_size_queue<IndexType, false> new_path(dev_car_paths + path_offset,
                                                car_path.capacity());

    gpuErrchk(cudaMemcpy((void*) &dev_Car[i].path(), (void*) &new_path,
               sizeof(fixed_size_queue<IndexType, false>),
               cudaMemcpyHostToDevice));
  }

  simulation::aos_int::s_size_inactive_cars = simulation::aos_int::s_size_Car;
  MEMCPY_TO_DEVICE(IndexType, inactive_cars);
  simulation::aos_int::s_size_inactive_cars = 0;
  gpuErrchk(cudaMemcpyToSymbol(s_size_inactive_cars,
                     &simulation::aos_int::s_size_inactive_cars,
                     sizeof(IndexType)));

  Simulation* dev_simulation;
  gpuErrchk(cudaMalloc((void**) &dev_simulation, sizeof(Simulation)));
  gpuErrchk(cudaMemcpyToSymbol(instance, &dev_simulation,
                     sizeof(Simulation*)));
  gpuErrchk(cudaMemcpy((void*) dev_simulation,
                       (void*) simulation::aos_int::instance,
                       sizeof(Simulation), cudaMemcpyHostToDevice));

  IndexType* h_car_reorder = new IndexType[simulation::aos_int::s_size_Car];
  for (int i = 0; i < simulation::aos_int::s_size_Car; ++i) {
    h_car_reorder[i] = i;
  }
  cudaMalloc((void**) &d_car_reorder, sizeof(IndexType)*simulation::aos_int::s_size_Car);
  cudaMemcpyToSymbol(s_Car_reorder, &d_car_reorder, sizeof(IndexType*));
  cudaMemcpy(d_car_reorder, h_car_reorder, sizeof(IndexType)*simulation::aos_int::s_size_Car,
             cudaMemcpyHostToDevice);
  cudaMalloc((void**) &d_car_reorder_keys, sizeof(IndexType)*simulation::aos_int::s_size_Car);
  cudaMemcpyToSymbol(s_Car_reorder_keys, &d_car_reorder_keys, sizeof(IndexType*));

  cudaMalloc((void**) &d_car_reorder_in, sizeof(IndexType)*simulation::aos_int::s_size_Car);
  cudaMemcpyToSymbol(s_Car_reorder_in, &d_car_reorder_in, sizeof(IndexType*));

  cudaMalloc((void**) &d_car_reorder_keys_in, sizeof(IndexType)*simulation::aos_int::s_size_Car);
  cudaMemcpyToSymbol(s_Car_reorder_keys_in, &d_car_reorder_keys_in, sizeof(IndexType*));

  Car* d_tmp_car;
  cudaMalloc((void**) &d_tmp_car, sizeof(Car)*simulation::aos_int::s_size_Car);
  cudaMemcpyToSymbol(s_tmp_Car, &d_tmp_car, sizeof(Car*));

  gpuErrchk(cudaDeviceSynchronize());
}

#undef MEMCPY_TO_DEVICE

#define PHYSICAL_REORDER true

__global__ void step_velocity() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    int realid = PHYSICAL_REORDER ? id : s_Car_reorder[id];
    if (s_Car[realid].is_active()) {
      s_Car[realid].step_velocity();
    }
  }
}

__global__ void step_assert_check_velocity() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    int realid = PHYSICAL_REORDER ? id : s_Car_reorder[id];
    if (s_Car[realid].is_active()) {
      s_Car[realid].assert_check_velocity();
    }
  }
}

__global__ void step_move() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    int realid = PHYSICAL_REORDER ? id : s_Car_reorder[id];
    if (s_Car[realid].is_active()) {
      s_Car[realid].step_move();
    }
  }
}

__global__ void step_reactivate() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    int realid = PHYSICAL_REORDER ? id : s_Car_reorder[id];
    s_Car[realid].step_reactivate();
  }
}

__global__ void step_traffic_lights() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_TrafficLight) {
    s_TrafficLight[id].step();
  }
}

__global__ void step_priority_ctrl() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_PriorityYieldTrafficController) {
    s_PriorityYieldTrafficController[id].step();
  }
}

__global__ void step_prepare_reorder() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    s_Car_reorder_in[id] = id;
    s_Car_reorder_keys_in[id] = s_Car[id].velocity();
//    s_Car_reorder_keys_in[id] = (s_Car[id].velocity() + 1) * s_size_Car + id;
//    s_Car_reorder_keys_in[id] = s_Cell[s_Car[id].position()].type();
//    s_Car_reorder_keys_in[id] = s_Cell[s_Car[id].position()].type()* s_size_Car + id;
//    s_Car_reorder_keys_in[id] = s_Car[id].rand32();

  }
}

__global__ void step_physical_reorder() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    //cudaMemcpy(&s_tmp_Car[id], &s_Car[s_Car_reorder[id]], sizeof(Car), cudaMemcpyDeviceToDevice);
    //memcpy(&s_tmp_Car[id], &s_Car[s_Car_reorder[id]], sizeof(Car));
    int* target = (int*) (&s_tmp_Car[id]);
    int* from = (int*) (&s_Car[id]);
    for (int i = 0; i < sizeof(Car)/sizeof(int); ++i) {
      *(target + i) = *(from + i);
    }
  }
}

__global__ void step_swap_car_arrays() {
  Car* tmp = s_Car;
  s_Car = s_tmp_Car;
  s_tmp_Car = tmp;
}

__device__ uint64_t s_checksum;

__global__ void checksum_kernel() {
  s_checksum = instance->checksum();
}

__global__ void step_random_state() {
  instance->step_random_state();
}

uint64_t checksum() {
  checksum_kernel<<<1, 1>>>();
  uint64_t result;
  cudaMemcpyFromSymbol(&result, s_checksum, sizeof(uint64_t));
  return result;
}

#define BLOCK_S 768

void step_reorder() {
  IndexType num_cars = simulation::aos_int::s_size_Car;
  step_prepare_reorder<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();

  // Determine temporary device storage requirements
  void     *d_temp_storage = NULL;
  size_t   temp_storage_bytes = 0;
  cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
      d_car_reorder_keys_in, d_car_reorder_keys, d_car_reorder_in, d_car_reorder, num_cars);
  // Allocate temporary storage
  cudaMalloc(&d_temp_storage, temp_storage_bytes);
  // Run sorting operation
  cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
      d_car_reorder_keys_in, d_car_reorder_keys, d_car_reorder_in, d_car_reorder, num_cars);
  // d_keys_out            <-- [0, 3, 5, 6, 7, 8, 9]
  // d_values_out          <-- [5, 4, 3, 1, 2, 0, 6]


  // Now physical reordering
  if (PHYSICAL_REORDER) {
    step_physical_reorder<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();
    step_swap_car_arrays<<<1, 1>>>();
  }

  gpuErrchk(cudaDeviceSynchronize());
}

void step() {
  printf("CAR SIZE: %i\n", sizeof(Car));

  IndexType num_cars = simulation::aos_int::s_size_Car;
  IndexType num_traffic_lights = simulation::aos_int::s_size_TrafficLight;
  IndexType num_priority_ctrl =
      simulation::aos_int::s_size_PriorityYieldTrafficController;

  auto t1 = std::chrono::steady_clock::now();
  unsigned long reorder_time =0;

  for (int i = 0; i < 1000; ++i) {
    step_random_state<<<1, 1>>>();
    step_traffic_lights<<<num_traffic_lights / BLOCK_S + 1, BLOCK_S>>>();
    step_priority_ctrl<<<num_priority_ctrl / BLOCK_S + 1, BLOCK_S>>>();
    step_velocity<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();

#ifndef NDEBUG
    step_assert_check_velocity<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();
    gpuErrchk(cudaDeviceSynchronize());
#endif

    step_move<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();
    step_reactivate<<<num_cars / BLOCK_S + 1, BLOCK_S>>>();

    if (false && i % 5 == 0 ) {
      auto t3 = std::chrono::steady_clock::now();
      step_reorder();
      auto t4 = std::chrono::steady_clock::now();

      reorder_time += std::chrono::duration_cast<std::chrono::milliseconds>(
        t4 - t3).count();
    }
#ifndef NDEBUG
    gpuErrchk(cudaDeviceSynchronize());
#else
    cudaDeviceSynchronize();
#endif  // NDEBUG
  }

  auto t2 = std::chrono::steady_clock::now();
  unsigned long millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      t2 - t1).count();
  auto cs = checksum();

  printf("Checksum: %lu, GPU Time (millis): %lu, Reorder time: %lu\n", cs, millis,reorder_time);
}

__device__ void Simulation::add_inactive_car(IndexType car) {
  s_inactive_cars[s_size_inactive_cars++] = car;
}

__device__ void Simulation::step_traffic_controllers() {
  assert(false);
}

__device__ IndexType Cell::num_outgoing_cells() const {
  return num_outgoing_cells_;
}

__device__ IndexType Cell::outgoing_cell(IndexType index) const {
  return s_outgoing_cells[first_outgoing_cell_idx_ + index];
}

__device__ IndexType Cell::num_incoming_cells() const {
  return num_incoming_cells_;
}

__device__ IndexType Cell::incoming_cell(IndexType index) const {
  return s_incoming_cells[first_incoming_cell_idx_ + index];
}

__device__ IndexType SharedSignalGroup::num_cells() const {
  return num_cells_;
}

__device__ IndexType SharedSignalGroup::cell(IndexType index) const {
  return s_signal_group_cells[first_cell_idx_ + index];
}

__device__ IndexType TrafficLight::num_signal_groups() const {
  return num_signal_groups_;
}

__device__ IndexType TrafficLight::signal_group(IndexType index) const {
  return s_traffic_light_signal_groups[first_signal_group_idx_ + index];
}

__device__ IndexType PriorityYieldTrafficController::num_groups() const {
  return num_groups_;
}

__device__ IndexType PriorityYieldTrafficController::group(IndexType index)
    const {
  return s_priority_ctrl_signal_groups[first_group_idx_ + index];
}

// Accessor methods for cars.
__device__ IndexType Simulation::num_cars() const { return s_size_Car; }
__device__ IndexType Simulation::car(IndexType index) const { return index; }

// Accessor methods for cells.
__device__ IndexType Simulation::num_cells() const { return s_size_Cell; }
__device__ IndexType Simulation::cell(IndexType index) const { return index; }

// Logic for traffic flow simulation.
#include "traffic_logic.inc"
#include "option_undo.inc"

}  // namespace aos_int
}  // namespace simulation
