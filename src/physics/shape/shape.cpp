#include "shape.h"

namespace pe_physics_shape {

std::atomic<uint32_t> Shape::_globalIdCounter(0);

Shape::Shape(): _global_id(++_globalIdCounter), _volume(PE_R(0.0)) {}

} // namespace pe_physics_shape
