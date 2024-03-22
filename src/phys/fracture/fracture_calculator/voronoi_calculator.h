#pragma once

#include "phys/fracture/fracture_utils/fracture_data_manager.h"
#include "phys/phys_general.h"
#include "utils/hash_vector.h"
#include "phys/fracture/fracture_utils/fracture_utils.h"
#include <algorithm>

namespace pe_phys_fracture  {

    /**
     * @brief A class to calculate the 3d delaunay triangulation
     * of a set of points.
     *
     * Based on the Bowyer-Watson algorithm.
     * process:
     * 1. add a bounding box to the diagram;
     * 2. sequentially add each point to the diagram:
     *   2.1. find all tetrahedrons that don't cater to the Delaunay condition;
     *   2.2. find all triangles that are supposed to be removed;
     *   2.3. add the new point, and corresponding triangles and tetrahedrons;
     * 3. remove the bounding box.
     */
    class VoronoiCalculator {
    private:
        FractureDataManager _manager{};
        pe::Array<pe::HashList<uint32_t>> _adjacency_list;

        void add_bounding_box(const pe::Array<pe::Vector3>& points);
        void remove_bounding_box();
        void generate(const pe::Array<pe::Vector3>& points);
        void calc_adjacency_list();

    public:
        void triangulate(const pe::Array<pe::Vector3>& points);
        uint32_t point_count() const { return _manager.vertex_count(); }
        pe::Vector3 get_point(uint32_t idx) { return _manager.get_vertex(idx).pos; }
        pe::Array<uint32_t> get_adjacent_points(uint32_t idx) { return _adjacency_list[idx].to_vector(); }
    };

} // namespace pe_phys_fracture