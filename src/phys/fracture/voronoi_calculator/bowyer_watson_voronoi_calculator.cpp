#include "bowyer_watson_voronoi_calculator.h"

namespace pe_phys_fracture {

    void BowyerWatsonVoronoiCalculator::add_bounding_box(const pe::Array<pe::Vector3>& points) {
        // find aabb bounds
        pe::Vector3 _min(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX);
        pe::Vector3 _max(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN);
        for (auto& p : points) {
            _min = PE_VEC_MIN2(_min, p);
            _max = PE_VEC_MAX2(_max, p);
        }

        // dilate to include all possible points
        const auto center = (_min + _max) / 2;
        const pe::Vector3 gap(0.1, 0.1, 0.1);
        _min = center + (_min - center) * 3 - gap;
        _max = center + (_max - center) * 3 + gap;

        // add bounding points
        const uint32_t v1i = _manager.add_vertex(_min);
        const uint32_t v2i = _manager.add_vertex({ _max.x(), _max.y(), _min.z() });
        const uint32_t v3i = _manager.add_vertex({ _max.x(), _min.y(), _max.z() });
        const uint32_t v4i = _manager.add_vertex({ _min.x(), _max.y(), _max.z() });

        // add bounding triangles
        const uint32_t t1i = _manager.add_triangle(v1i, v2i, v3i);
        const uint32_t t2i = _manager.add_triangle(v2i, v1i, v4i);
        const uint32_t t3i = _manager.add_triangle(v3i, v2i, v4i);
        const uint32_t t4i = _manager.add_triangle(v3i, v4i, v1i);

        // add bounding tetrahedrons
        _manager.add_tetrahedron(t1i, t2i, t3i, t4i, v1i, v2i, v3i, v4i);
    }

    void BowyerWatsonVoronoiCalculator::remove_bounding_box() {
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

    void BowyerWatsonVoronoiCalculator::generate(const pe::Array<pe::Vector3>& points) {
        // generate triangle mesh for all points, using Bowyer-Watson algorithm
        for (auto& point : points) {
            // find all tetrahedrons that don't cater to the Delaunay condition
            pe::Array<uint32_t> bad_tetrahedrons;
            const uint32_t tet_count = _manager.tetrahedron_count();
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
                    if (good_triangles.find(tri_id) != good_triangles.end()) {
                        good_triangles.erase(good_triangles.find(tri_id));
                        bad_triangles.push_back(tri_id);
                    } else {
                        good_triangles.push_back(tri_id);
                    }
                }
            }

            // add new vertices, triangles and tetrahedrons
            const auto vi = _manager.add_vertex(point);
            for (const auto tri_id : good_triangles) {
                const auto tri = _manager.get_triangle(tri_id);
                const auto tri_id1 = _manager.add_triangle(vi, tri.vert_ids[0], tri.vert_ids[1]);
                const auto tri_id2 = _manager.add_triangle(vi, tri.vert_ids[1], tri.vert_ids[2]);
                const auto tri_id3 = _manager.add_triangle(vi, tri.vert_ids[2], tri.vert_ids[0]);
                _manager.add_tetrahedron(tri_id, tri_id1,tri_id2, tri_id3,
                                         vi, tri.vert_ids[0], tri.vert_ids[1], tri.vert_ids[2]);
            }

            // remove bad tetrahedrons
            for (uint32_t i = UI(bad_tetrahedrons.size()) - 1; i != -1; i--) {
                _manager.remove_tetrahedron(bad_tetrahedrons[i]);
            }

            // remove bad triangles
            std::sort(bad_triangles.begin(), bad_triangles.end());
            for (uint32_t i = UI(bad_triangles.size()) - 1; i != -1; i--) {
                _manager.remove_triangle(bad_triangles[i]);
            }
        }
    }

    void BowyerWatsonVoronoiCalculator::triangulate(const pe::Array<pe::Vector3>& points) {
        _manager.clear();
        _adjacency_list.clear();
        add_bounding_box(points);
        generate(points);
        remove_bounding_box();
        calc_adjacency_list();
    }

} // namespace pe_phys_fracture