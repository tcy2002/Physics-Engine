#pragma once

#include "voronoi_calculator.h"
#include "phys/phys_general.h"
#include "fracture_utils/fracture_utils.h"
#include "utils/hash_vector.h"

namespace pe_phys_fracture {

    /**
     * @brief The core class to calculate the fracture of a mesh.
     *
     * Input: a simple mesh, positions of fragments
     * Output: fragment meshes
     *
     * Current algorithm only support convex mesh, no limitation
     * on other aspects.
     *
     * Calculation process:
     * 1. triangulate the fragment positions into voronoi diagram;
     * 2. read and transform mesh data from triangles into faces;
     * 3. cut the origin mesh by each voronoi cell:
     *   3.1. sequentially cut by each median plane:
     *     3.1.1. cut each face of the mesh;
     *     3.1.2. add a new face of the cutting plane;
     *   3.2. regenerate triangles from faces;
     *   3.3. output the new mesh of the cell.
     * Be aware: input and output mesh is stored in simple_mesh,
     * which needs to be transformed from other format in advance.
     */
    class FractureCalculator {
    private:
        VoronoiCalculator _voronoi{};

        void cut_mesh(const pe::Mesh& mesh, pe::Array<pe::Mesh>& new_meshes);
        void cut_one_mesh(const FractureDataManager& mesh, uint32_t idx, FractureDataManager& new_mesh);
        static void cut_mesh_by_plane(FractureDataManager& old_mesh, const pe::Vector3& p, const pe::Vector3& n,
                                      FractureDataManager& new_mesh);
        static pe::Array<pe::Vector3> cut_face_by_plane(uint32_t face_id, FractureDataManager& old_mesh,
                                                          const pe::Vector3& p, const pe::Vector3& n,
                                                          FractureDataManager& new_mesh);

    public:
        void triangulate(const pe::Array<pe::Vector3>& points) { _voronoi.triangulate(points); }
        void fracture(const pe::Mesh& mesh, pe::Array<pe::Mesh>& new_meshes) { cut_mesh(mesh, new_meshes); }
    };

} // namespace pe_phys_fracture