// Option AOS INT: Use class-based AOS implementation no GUI output.

#include <limits>

using IndexType = unsigned int;
const IndexType kMaxIndexType = std::numeric_limits<IndexType>::max();

#undef OPTION_STANDARD
#define PTR(class) IndexType
#define CPTR(class) const IndexType
#define THIS id_
#define METHOD_CALL(class, ptr, name, ...) s_ ## class [ptr].name(__VA_ARGS__)
#define DEVICE_MODE __device__
