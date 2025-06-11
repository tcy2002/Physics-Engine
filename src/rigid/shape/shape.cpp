#include "shape.h"

namespace pe_phys_shape {

    std::atomic<uint32_t> Shape::_globalIdCounter(0);

    Shape::Shape(): _global_id(++_globalIdCounter), _volume(0) {}

} // namespace pe_phys_shape
