#include "fracture_solver.h"
#include "phys/fracture/voronoi_calculator/bowyer_watson_voronoi_calculator.h"

namespace pe_phys_fracture {

    class BisectFractureSolver : public FractureSolver {
    protected:
        VoronoiCalculator* _voronoi;

    public:
        BisectFractureSolver() { _voronoi = new BowyerWatsonVoronoiCalculator(); }
        virtual ~BisectFractureSolver() { delete _voronoi;}

        virtual void solve(const pe::Array<FractureSource>& sources) override;
    };

} // namespace pe_phys_fracture