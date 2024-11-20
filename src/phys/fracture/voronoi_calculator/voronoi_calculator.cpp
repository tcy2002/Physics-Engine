#include "voronoi_calculator.h"

namespace pe_phys_fracture {

    void VoronoiCalculator::calc_adjacency_list() {
        // get adjacency list according to triangles
        const uint32_t vert_count = _manager.vertex_count();
        if (vert_count == 0) {
            return;
        }

        for (uint32_t i = 0; i < vert_count; i++) {
            _adjacency_list.emplace_back(vert_count);
        }

        const uint32_t tri_count = _manager.triangle_count();
        for (uint32_t i = 0; i < tri_count; i++) {
            const auto t = _manager.get_triangle(i);
            _adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[1]);
            _adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[2]);
            _adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[0]);
            _adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[2]);
            _adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[0]);
            _adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[1]);
        }
    }

} // namespace pe_phys_fracture