#include "voronoi_calculator.h"

namespace pe_phys_fracture {

    void VoronoiCalculator::add_bounding_box(const pe::Array<pe::Vector3>& points) {
        // find aabb bounds
        pe::Vector3 _min(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX);
        pe::Vector3 _max(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN);
        for (auto& p : points) {
            _min = min_vec(_min, p);
            _max = max_vec(_max, p);
        }

        // dilate to include all possible points
        auto center = (_min + _max) / 2;
        pe::Vector3 gap(.1, .1, .1);
        _min = center + (_min - center) * 3 - gap;
        _max = center + (_max - center) * 4 + gap;

        // add bounding box points
        uint32_t v1i = _manager.add_vertex(_min);
        uint32_t v2i = _manager.add_vertex({_max.x, _max.y, _min.z});
        uint32_t v3i = _manager.add_vertex({_max.x, _min.y, _max.z});
        uint32_t v4i = _manager.add_vertex({_min.x, _max.y, _max.z});

        // add bounding box triangles
        uint32_t t1i = _manager.add_triangle(v1i, v2i, v3i);
        uint32_t t2i = _manager.add_triangle(v2i, v1i, v4i);
        uint32_t t3i = _manager.add_triangle(v3i, v2i, v4i);
        uint32_t t4i = _manager.add_triangle(v3i, v4i, v1i);

        // add bounding box tetrahedrons
        _manager.add_tetrahedron(t1i, t2i, t3i, t4i, v1i, v2i, v3i, v4i);
    }

    void VoronoiCalculator::remove_bounding_box() {
        // clear the tetrahedrons
        _manager.clear_tetrahedrons();

        // remove triangles that contain bounding box points
        for (uint32_t i = _manager.triangle_count() - 1; i != -1; i--) {
            auto tri = _manager.get_triangle(i);
            if (tri.vert_ids[0] < 4 || tri.vert_ids[1] < 4 || tri.vert_ids[2] < 4) {
                _manager.remove_triangle(i);
            }
        }

        // remove bounding box points
        for (uint32_t i = 0; i < 4; i++) {
            _manager.remove_vertex(0);
        }
    }

    void VoronoiCalculator::generate(const pe::Array<pe::Vector3>& points) {
        // generate triangle mesh for all points, using Bowyer-Watson algorithm
        for (auto& point : points) {
            // find all tetrahedrons that don't cater to the Delaunay condition
            pe::Array<uint32_t> bad_tetrahedrons;
            uint32_t tet_count = _manager.tetrahedron_count();
            for (uint32_t i = 0; i < tet_count; i++) {
                auto tet = _manager.get_tetrahedron(i);
                if (is_point_inside_sphere(point, tet.center, tet.radius)) {
                    bad_tetrahedrons.push_back(i);
                }
            }

            // find all triangles that are part of the boundary of the cavity
            // or should be removed
            pe::Uint32HashList good_triangles(_manager.triangle_count() * 2);
            pe::Array<uint32_t> bad_triangles;
            for (auto tet_id : bad_tetrahedrons) {
                auto tet = _manager.get_tetrahedron(tet_id);
                for (auto tri_id : tet.tri_ids) {
                    if (good_triangles.contains(tri_id)) {
                        good_triangles.erase(tri_id);
                        bad_triangles.push_back(tri_id);
                    } else {
                        good_triangles.push_back(tri_id);
                    }
                }
            }

            // add new vertices, triangles and tetrahedrons
            auto vi = _manager.add_vertex(point);
            for (auto tri_id : good_triangles) {
                auto tri = _manager.get_triangle(tri_id);
                auto tri_id1 = _manager.add_triangle(vi, tri.vert_ids[0], tri.vert_ids[1]);
                auto tri_id2 = _manager.add_triangle(vi, tri.vert_ids[1], tri.vert_ids[2]);
                auto tri_id3 = _manager.add_triangle(vi, tri.vert_ids[2], tri.vert_ids[0]);
                _manager.add_tetrahedron(tri_id, tri_id1,tri_id2, tri_id3,
                                         vi, tri.vert_ids[0], tri.vert_ids[1], tri.vert_ids[2]);
            }

            // remove bad tetrahedrons
            for (uint32_t i = (uint32_t)bad_tetrahedrons.size() - 1; i != -1; i--) {
                _manager.remove_tetrahedron(bad_tetrahedrons[i]);
            }

            // remove bad triangles
            std::sort(bad_triangles.begin(), bad_triangles.end());
            for (uint32_t i = (uint32_t)bad_triangles.size() - 1; i != -1; i--) {
                _manager.remove_triangle(bad_triangles[i]);
            }
        }
    }

    void VoronoiCalculator::calc_adjacency_list() {
        // get adjacency list according to triangles
        _adjacency_list.clear();
        uint32_t vert_count = _manager.vertex_count();
        if (vert_count == 0) {
            return;
        }

        for (uint32_t i = 0; i < vert_count; i++) {
            _adjacency_list.emplace_back(vert_count);
        }

        uint32_t tri_count = _manager.triangle_count();
        for (uint32_t i = 0; i < tri_count; i++) {
            auto t = _manager.get_triangle(i);
            _adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[1]);
            _adjacency_list[t.vert_ids[0]].push_back(t.vert_ids[2]);
            _adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[0]);
            _adjacency_list[t.vert_ids[1]].push_back(t.vert_ids[2]);
            _adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[0]);
            _adjacency_list[t.vert_ids[2]].push_back(t.vert_ids[1]);
        }
    }

    void VoronoiCalculator::triangulate(const pe::Array<pe::Vector3>& points) {
        _manager.clear();
        add_bounding_box(points);
        generate(points);
        remove_bounding_box();
        calc_adjacency_list();
    }

} // namespace pe_phys_fracture