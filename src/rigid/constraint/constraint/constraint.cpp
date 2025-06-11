#include "constraint.h"

namespace pe_phys_constraint {

    std::atomic<uint32_t> Constraint::_globalIdCounter(0);

    Constraint::Constraint(): _global_id(++_globalIdCounter) {}

} // namespace pe_phys_constraint