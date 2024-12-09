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
                pe::VectorX diff = (out - in) - l * g;
                if (diff.norm() < 1e-10 && PE_ABS(c_diff) < 1e-10) {
                    break;
                }
                pe::MatrixMN h = pe::MatrixMN::Identity(in.size(), in.size()) - l * hessian(out);
                pe::MatrixMN lhs(in.size() + 1, in.size() + 1);
                pe::VectorX rhs(in.size() + 1);
                lhs << h, -g, -g.transpose(), 0;
                rhs << -diff, -c_diff;
                lhs.block(0, 0, in.size(), in.size()) += pe::MatrixMN::Identity(in.size(), in.size()) * diff.norm();
                Eigen::JacobiSVD<pe::MatrixMN> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);
                pe::VectorX x = svd.solve(rhs);
                pe::Real dl = x(x.size() - 1);
                pe::VectorX dx = x.block(0, 0, in.size() - 1, 1);
                pe::Real step = 1.0;
                for (int j = 0; j < 30; j++) {
                    pe::VectorX test_out = out + step * dx;
                    pe::Real test_l = l + step * dl;
                    pe::Real test_c = constraint(test_out);
                    pe::Real test_c_diff = target - test_c;
                    pe::VectorX test_g = gradient(test_out);
                    pe::VectorX test_diff = (test_out - in) - test_l * test_g;
                    if (test_diff.squaredNorm() + test_c_diff * test_c_diff < diff.squaredNorm() + c_diff * c_diff) {
                        out = test_out;
                        l = test_l;
                        break;
                    }
                    step *= 0.5;
                }
            }
            return out;
        }
    };

} // namespace pe_phys_constraint