#include "fracture_calculator.h"

namespace pe_phys_fracture {

    void FractureCalculator::cut_mesh(const pe::Mesh& mesh, std::vector<pe::Mesh>& new_meshes) {
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
        utils::hash_vector<pe::Vector3> inter_points(FRAC_VEC_INIT(50));

        // cut all the faces
        for (uint32_t face_id = 0; face_id < face_count; face_id++) {
            inter_points.append(cut_face_by_plane(face_id, old_mesh, p, n, new_mesh));
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

} // namespace pe_phys_fracture