#include "fracture_utils.h"

namespace pe_phys_fracture {

    pe::Matrix3 from_two_vectors(const pe::Vector3& a, const pe::Vector3& b) {
        const pe::Vector3 c = a.cross(b);
        const pe::Real s = c.norm();
        const pe::Real c_ = a.dot(b);
        pe::Matrix3 c_hat = pe::Matrix3::zeros();
        c_hat[0][1] = -c.z;
        c_hat[0][2] = c.y;
        c_hat[1][0] = c.z;
        c_hat[1][2] = -c.x;
        c_hat[2][0] = -c.y;
        c_hat[2][1] = c.x;
        return pe::Matrix3::identity() + c_hat + c_hat * c_hat * (1 - c_) / (s * s);
    }

    void calc_tet_bounding_sphere(const pe::Vector3& v1, const pe::Vector3& v2,
                                  const pe::Vector3& v3, const pe::Vector3& v4,
                                  pe::Vector3& center, pe::Real& radius) {
        const pe::Vector3 v1v2 = v2 - v1, v1v3 = v3 - v1, v1v4 = v4 - v1;
        const pe::Vector3 v1v2m = (v1 + v2) / 2, v1v3m = (v1 + v3) / 2, v1v4m = (v1 + v4) / 2;
        const pe::Vector3 v1v2v3n = v1v2.cross(v1v3), v1v3v4n = v1v3.cross(v1v4);
        const pe::Vector3 v1v2n = v1v2.cross(v1v2v3n), v1v3n = v1v3.cross(v1v3v4n);

        //TODO: should check if the tetrahedron is degenerated?
        const pe::Real u1 = v1v2n.dot(v1v3);
        const pe::Real u2 = v1v3n.dot(v1v4);
        const pe::Real u3 = v1v2v3n.dot(v1v4);

        const pe::Real k1 = v1v3.dot(v1v3m - v1v2m) / u1;
        const pe::Real k2 = v1v4.dot(v1v4m - v1v3m) / u2;

        const auto p1 = v1v2m + v1v2n * k1;
        const auto p2 = v1v3m + v1v3n * k2;

        const pe::Real k3 = v1v4.dot(p2 - p1) / u3;

        center = p1 + v1v2v3n * k3;
        radius = (center - v1).norm();
    }

    bool calc_line_plane_intersection(const pe::Vector3& p, const pe::Vector3& n,
                                      const pe::Vector3& v1, const pe::Vector3& v2,
                                      pe::Vector3& inter, pe::Real& t) {
        const pe::Real d = n.dot(v2 - v1);
        if (PE_APPROX_EQUAL(d, 0)) {
            return false;
        }

        t = n.dot(p - v1) / d;
        inter = v1 + (v2 - v1) * t;
        return t > PE_EPS && t < 1 - PE_EPS;
    }

    bool is_point_upside_plane(const pe::Vector3& p, const pe::Vector3& n, const pe::Vector3& v) {
        return n.dot(v - p) > PE_EPS;
    }

    bool is_point_on_plane(const pe::Vector3& p, const pe::Vector3& n, const pe::Vector3& v) {
        return PE_APPROX_EQUAL(n.dot(v - p), 0);
    }

    bool is_point_inside_sphere(const pe::Vector3& p, const pe::Vector3& center, pe::Real radius) {
        return (p - center).norm() < radius - PE_EPS;
    }

    bool are_points_collinear(const pe::Vector3& p1, const pe::Vector3& p2, const pe::Vector3& p3) {
        return PE_APPROX_EQUAL((p1 - p2).cross(p3 - p2).norm(), 0);
    }

} // namespace pe_phys_fracture