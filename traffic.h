
template<typename S>
class Simulation {
 private:
  template<typename Type>
  using Ptr = typename S::template Ptr<Type>;

 public:
  class Car;

  class Cell {
   private:
    using CellVec =
        typename S::template Vector<Ptr<Cell>>;

   public:
    // Returns the car that occupies this cell.
    __host__ __device__ Ptr<Car> car() const { return car_; }

    __host__ __device__ IndexType id() const { return id_; }

    __host__ __device__ const CellVec& incoming_cells() const {
      return incoming_cells_;
    }

    // Returns true if the cell is free.
    __host__ __device__ bool is_free() const { return is_free_; }

    // Returns true if this cell is a sink.
    __host__ __device__ bool is_sink() const { return is_sink_; }

    // Returns the maximum velocity allowed on this cell at this moment. This
    // function takes into account velocity limitations due to traffic lights.
    __host__ __device__ int max_velocity() const;

    // A car enters this cell.
    __host__ __device__ void occupy(Ptr<Car> car);

    __host__ __device__ const CellVec& outgoing_cells() const {
      return outgoing_cells_;
    }

    // The current car on this cell leaves the cell.
    __host__ __device__ void release();

    // Removes the maximum temporary speed limit.
    __host__ __device__ void remove_controller_max_velocity() {
      controller_max_velocity_ = max_velocity_;
    }

    // Sets the maximum temporary speed limit (traffic controller).
    __host__ __device__ void set_controller_max_velocity(int velocity) {
      controller_max_velocity_ = velocity;
    }

    // Return max. velocity regardless of traffic controllers.
    __host__ __device__ int street_max_velocity() const;

    // Additional information can be stored in the tag.
    __host__ __device__ uint32_t tag() const { return tag_; }

    // Returns the type of this cell.
    __host__ __device__ Type type() const { return type_; }

    // Return x and y coordinates of this cell.
    __host__ __device__ double x() const { return x_; }
    __host__ __device__ double y() const { return y_; }

   private:
    typename S::IndexType id_;

    // A list of all incoming and outgoing cells, forming the street network.
    const CellVec incoming_cells_, outgoing_cells_;

    const Type type_;

    bool is_free_;

    // Sinks do not have any outgoing cells and cars are automatically removed
    // from them at the end of an iteration.
    const bool is_sink_;

    // The maximum velocity allowed on this cell.
    const int max_velocity_;

    // Controller may impose a higher or lower speed limit on this cell.
    int controller_max_velocity_;

    // Pointer to the car that is located on this cell.
    Ptr<Car> car_;

    // Coordinates of this cell. Used for debugging purposes and to calculate
    // the checksum.
    const double x_, y_;

    // Additional information may be stored in this tag.
    uint32_t tag_;
  };
};
