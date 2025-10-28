#pragma once

#include "voronoi_calculator.h"

namespace pe_phys_fracture  {

    class BowyerWatsonVoronoiCalculator : public VoronoiCalculator {
    protected:
        void add_bounding_box(const pe::Array<pe::Vector3>& points);
        void remove_bounding_box();
        void generate(const pe::Array<pe::Vector3>& points);

    public:
        BowyerWatsonVoronoiCalculator() {}
        virtual ~BowyerWatsonVoronoiCalculator() {}

        PE_API void triangulate(const pe::Array<pe::Vector3>& points) override;
    };

} // namespace pe_phys_fracture