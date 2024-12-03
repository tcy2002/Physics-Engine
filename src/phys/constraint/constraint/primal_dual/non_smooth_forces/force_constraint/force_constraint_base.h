#pragma once

namespace pe_phys_constraint {

    class ForceConstraintBase {
    public:
        ForceConstraintBase() {}
        virtual ~ForceConstraintBase() {}
        virtual pe::Real constraint(const pe::VectorX& in) const = 0;
        virtual pe::VectorX gradient(const pe::VectorX& in) const = 0;
        virtual pe::MatrixMN hessian(const pe::VectorX& in) const = 0;
        virtual pe::Real radius_at(pe::Real i0) const { return 0; }
        virtual pe::VectorX project(const pe::VectorX& in, const pe::VectorX& start, pe::Real target = 0) const {
            pe::VectorX out = start;
            pe::Real l = 0;
            for (int i = 0; i < 30; i++) {
                pe::Real c = constraint(out);
                pe::Real c_diff = target - c;
                pe::VectorX g = gradient(out);
                pe::VectorX diff = (out - in) - g * l;
                if (diff.norm() < 1e-10 && PE_ABS(c_diff) < 1e-10) {
                    break;
                }
                pe::MatrixMN h = pe::MatrixMN::Identity(in.size(), in.size()) - l * hessian(out);
                pe::MatrixMN lhs(in.size() + 1, in.size() + 1);
                pe::VectorX rhs(in.size() + 1);
                for (size_t j = 0; j < in.size(); j++) {
                    for (size_t k = 0; k < in.size(); k++) {
                        lhs(j, k) = h(j, k);
                    }
                    lhs(j, in.size()) = -g[j];
                    lhs(in.size(), j) = -g[j];
                    rhs[j] = diff[j];
                }
                lhs(in.size(), in.size()) = 0;
                rhs[in.size()] = -c_diff;
                // TODO
            }
            return out;
        }
    };

} // namespace pe_phys_constraint