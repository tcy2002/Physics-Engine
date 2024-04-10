#include "fracture_solver.h"
#include "phys/fracture/voronoi_calculator/voronoi_calculator.h"

namespace pe_phys_fracture {

    class SimpleFractureSolver : public FractureSolver {
    protected:
        VoronoiCalculator _voronoi{};

        void cut_mesh(const pe::Mesh& mesh, pe::Array<pe::Mesh>& new_meshes);
        void cut_one_mesh(const FractureDataManager& mesh, uint32_t idx, FractureDataManager& new_mesh);
        static void cut_mesh_by_plane(FractureDataManager& old_mesh, const pe::Vector3& p, const pe::Vector3& n,
                                      FractureDataManager& new_mesh);
        static pe::Array<pe::Vector3> cut_face_by_plane(uint32_t face_id, FractureDataManager& old_mesh,
                                                        const pe::Vector3& p, const pe::Vector3& n,
                                                        FractureDataManager& new_mesh);

    public:
        virtual void solve(const pe::Array<FractureSource>& sources) override;
    };

} // namespace pe_phys_fracture