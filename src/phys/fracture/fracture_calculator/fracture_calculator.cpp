#include "fracture_calculator.h"

namespace pe_phys_fracture {

    void FractureCalculator::cut_mesh(const pe::Mesh& mesh, pe::Array<pe::Mesh>& new_meshes) {
        uint32_t point_count = _voronoi.point_count();
        FractureDataManager worker;
        worker.import_from_mesh(mesh);

        // generate the new meshes of each point
        for (uint32_t i = 0; i < point_count; i++) {
            FractureDataManager result;
            cut_one_mesh(worker, i, result);
            new_meshes.push_back({});
            result.export_to_mesh(new_meshes.back());
        }
    }

    void FractureCalculator::cut_one_mesh(const FractureDataManager &mesh, uint32_t idx,
                                          FractureDataManager &new_mesh) {
        auto point = _voronoi.get_point(idx);
        auto adjacent_point_ids = _voronoi.get_adjacent_points(idx);

        // cut the mesh by each adjacent point
        new_mesh = mesh;
        for (auto other_id : adjacent_point_ids) {
            auto other = _voronoi.get_point(other_id);
            auto center = (point + other) / 2;
            auto normal = (other - point).normalized();
            FractureDataManager result;
            cut_mesh_by_plane(new_mesh, center, normal, result);
            new_mesh = result;
        }
    }

    void FractureCalculator::cut_mesh_by_plane(FractureDataManager &old_mesh, const pe::Vector3 &p,
                                               const pe::Vector3 &n, FractureDataManager &new_mesh) {
        uint32_t face_count = old_mesh.face_count();
        pe::Vector3HashList inter_points(50);

        // cut all the faces
        for (uint32_t face_id = 0; face_id < face_count; face_id++) {
            for (auto& point : cut_face_by_plane(face_id, old_mesh, p, n, new_mesh)) {
                inter_points.push_back(point);
            }
        }
        if (inter_points.size() < 3) {
            return;
        }

        // add a new face of the cutting plane
        polygon new_face(n);
        auto sorted_points = inter_points.to_vector();
        std::sort(sorted_points.begin() + 1, sorted_points.end(),
                  [&](const pe::Vector3& a, const pe::Vector3& b) {
            return (a - sorted_points[0]).cross(b - sorted_points[0]).dot(n) > 0;
        });
        for (const auto& v : sorted_points) {
            new_face.add_vert(new_mesh.add_vertex(v, n));
        }
        new_mesh.add_face(new_face);
    }

    pe::Array<pe::Vector3> FractureCalculator::cut_face_by_plane(uint32_t face_id, FractureDataManager &old_mesh,
                                                                   const pe::Vector3 &p, const pe::Vector3 &n,
                                                                   FractureDataManager &new_mesh) {
        auto face = old_mesh.get_face(face_id);
        uint32_t vert_count = (uint32_t)face.vert_ids.size();
        pe::Vector3HashList inter_points(vert_count);
        pe::Array<vertex> vertices(vert_count);
        pe::Array<int> side(vert_count, -1);
        pe::Uint32HashList new_point_ids(vert_count * 2);

        // check the side of each vertex to the cutting plane
        for (uint32_t i = 0; i < vert_count; i++) {
            vertices[i] = old_mesh.get_vertex(face.vert_ids[i]);
            side[i] = is_point_on_plane(p, n, vertices[i].pos) ? 0 : -1;
            side[i] = is_point_upside_plane(p, n, vertices[i].pos) ? 1 : -1;
        }

        // traverse the segments in order, and add at most 2 intersection points
        for (uint32_t i = 0; i < vert_count; i++) {
            uint32_t j = (i + 1) % vert_count;
            side[i] == 0 && inter_points.push_back(vertices[i].pos);
            side[j] == 0 && inter_points.push_back(vertices[j].pos);

            i == 0 && side[i] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[i].pos, vertices[i].nor));

            pe::Vector3 inter;
            pe::Real t;
            if (side[i] * side[j] < 0) {
                calc_line_plane_intersection(p, n, vertices[i].pos, vertices[j].pos, inter, t);
                auto nor = (vertices[j].nor * t + vertices[i].nor * (1 - t)).normalized();
                new_point_ids.push_back(new_mesh.add_vertex(inter, nor));
                inter_points.push_back(inter);
            }

            j != 0 && side[j] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[j].pos, vertices[j].nor));
        }

        // some corner cases
        if (new_point_ids.size() < 3) {
            return inter_points.to_vector();
        }

        // add the remaining face downside the cutting plane
        polygon new_face;
        auto v1 = new_mesh.get_vertex(new_point_ids[0]).pos;
        auto v2 = new_mesh.get_vertex(new_point_ids[1]).pos;
        auto v3 = new_mesh.get_vertex(new_point_ids[2]).pos;
        new_face.nor = (v2 - v1).cross(v3 - v1).normalized();
        new_face.vert_ids = new_point_ids.to_vector();
        new_mesh.add_face(new_face);
        return inter_points.to_vector();
    }

} // namespace pe_phys_fracture