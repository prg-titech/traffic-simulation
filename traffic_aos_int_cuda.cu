#include <chrono>

#include "traffic_aos_int_cuda.h"
#include "random.h"

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
}

#undef MEMCPY_TO_DEVICE

__global__ void step_velocity() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    if (s_Car[id].is_active()) {
      s_Car[id].step_velocity();
    }
  }
}

__global__ void step_assert_check_velocity() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    if (s_Car[id].is_active()) {
      s_Car[id].assert_check_velocity();
    }
  }
}

__global__ void step_move() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    if (s_Car[id].is_active()) {
      s_Car[id].step_move();
    }
  }
}

__global__ void step_reactivate() {
  IndexType id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < s_size_Car) {
    s_Car[id].step_reactivate();
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

void step() {
  IndexType num_cars = simulation::aos_int::s_size_Car;
  IndexType num_traffic_lights = simulation::aos_int::s_size_TrafficLight;
  IndexType num_priority_ctrl =
      simulation::aos_int::s_size_PriorityYieldTrafficController;

  auto t1 = std::chrono::steady_clock::now();

  for (int i = 0; i < 1000; ++i) {
    step_random_state<<<1, 1>>>();
    step_traffic_lights<<<num_traffic_lights / 1024 + 1, 1024>>>();
    step_priority_ctrl<<<num_priority_ctrl / 1024 + 1, 1024>>>();
    step_velocity<<<num_cars / 1024 + 1, 1024>>>();

#ifndef NDEBUG
    step_assert_check_velocity<<<num_cars / 1024 + 1, 1024>>>();
    gpuErrchk(cudaDeviceSynchronize());
#endif

    step_move<<<num_cars / 1024 + 1, 1024>>>();
    step_reactivate<<<num_cars / 1024 + 1, 1024>>>();

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

  printf("Checksum: %lu, GPU Time (millis): %lu\n", cs, millis);
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
