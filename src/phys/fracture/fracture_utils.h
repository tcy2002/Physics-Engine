#pragma once

#include <vector>
#include "phys/phys_general.h"

namespace pe_phys_fracture {

    bool approx_equal(pe::Real a, pe::Real b) {
        return std::abs(a - b) < PE_EPS;
    }

    bool approx_equal(const pe::Vector3& a, const pe::Vector3& b) {
        return approx_equal(a.x, b.x) && approx_equal(a.y, b.y) && approx_equal(a.z, b.z);
    }

    pe::Vector3 max_vec(const pe::Vector3& a, const pe::Vector3& b) {
        return pe::Vector3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }

    pe::Vector3 min_vec(const pe::Vector3& a, const pe::Vector3& b) {
        return pe::Vector3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }

    pe::Vector3 average(const std::vector<pe::Vector3>& vecs) {
        pe::Vector3 sum;
        for (const auto& v : vecs) {
            sum += v;
        }
        return sum / (pe::Real)vecs.size();
    }

    pe::Matrix3 from_two_vectors(const pe::Vector3& a, const pe::Vector3& b) {
        pe::Vector3 c = a.cross(b);
        pe::Real s = c.norm();
        pe::Real c_ = a.dot(b);
        pe::Matrix3 c_hat = pe::Matrix3::zeros();
        c_hat[0][1] = -c.z;
        c_hat[0][2] = c.y;
        c_hat[1][0] = c.z;
        c_hat[1][2] = -c.x;
        c_hat[2][0] = -c.y;
        c_hat[2][1] = c.x;
        return pe::Matrix3::identity() + c_hat + c_hat * c_hat * (1 - c_) / (s * s);
    }

    pe::Real calc_mesh_volume(const pe::Mesh& mesh) {
        pe::Real volume = 0;
        for (size_t i = 0; i < mesh.faces.size(); i++) {
            const pe::Vector3& v0 = mesh.vertices[mesh.faces[i].indices[0]].position;
            const pe::Vector3& v1 = mesh.vertices[mesh.faces[i].indices[0]].position;
            const pe::Vector3& v2 = mesh.vertices[mesh.faces[i].indices[1]].position;
            volume += v0.dot(v1.cross(v2));
        }
        return volume / 6;
    }

    pe::Vector3 calc_approx_mesh_centroid(const std::vector<pe::Vector3>& verts, const std::vector<uint32_t>& indices) {
        pe::Vector3 centroid = pe::Vector3::zeros();
        pe::Real total_area = 0;
        for (size_t i = 0; i < indices.size(); i += 3) {
            const pe::Vector3& v0 = verts[indices[i]];
            const pe::Vector3& v1 = verts[indices[i + 1]];
            const pe::Vector3& v2 = verts[indices[i + 2]];
            pe::Vector3 face_center = (v0 + v1 + v2) / 3;
            pe::Real face_area = v0.cross(v1).dot(v2);
            centroid += face_center * face_area;
            total_area += face_area;
        }
        return centroid / total_area;
    }

    void calc_tet_bounding_sphere(const pe::Vector3 v1, const pe::Vector3 v2,
                                  const pe::Vector3 v3, const pe::Vector3 v4,
                                  pe::Vector3& center, pe::Real& radius) {
        pe::Vector3 v1v2 = v2 - v1, v1v3 = v3 - v1, v1v4 = v4 - v1;
        pe::Vector3 v1v2m = (v1 + v2) / 2, v1v3m = (v1 + v3) / 2, v1v4m = (v1 + v4) / 2;
        pe::Vector3 v1v2v3n = v1v2.cross(v1v3), v1v3v4n = v1v3.cross(v1v4);
        pe::Vector3 v1v2n = v1v2.cross(v1v2v3n), v1v3n = v1v3.cross(v1v3v4n);

        //TODO: should check if the tetrahedron is degenerated?
        pe::Real u1 = v1v2n.dot(v1v3);
        pe::Real u2 = v1v3n.dot(v1v4);
        pe::Real u3 = v1v2v3n.dot(v1v4);

        pe::Real k1 = v1v3.dot(v1v3m - v1v2m) / u1;
        pe::Real k2 = v1v4.dot(v1v4m - v1v3m) / u2;

        auto p1 = v1v2m + v1v2n * k1;
        auto p2 = v1v3m + v1v3n * k2;

        pe::Real k3 = v1v4.dot(p2 - p1) / u3;

        center = p1 + v1v2v3n * k3;
        radius = (center - v1).norm();
    }

    bool calc_line_plane_intersection(const pe::Vector3& p, const pe::Vector3& n,
                                      const pe::Vector3& v1, const pe::Vector3& v2,
                                      pe::Vector3& inter, pe::Real& t) {
        pe::Real d = n.dot(v2 - v1);
        if (approx_equal(d, 0)) {
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
        return approx_equal(n.dot(v - p), 0);
    }

    bool is_point_inside_sphere(const pe::Vector3& p, const pe::Vector3& center, pe::Real radius) {
        return (p - center).norm() < radius - PE_EPS;
    }

    bool are_points_collinear(const pe::Vector3& p1, const pe::Vector3& p2, const pe::Vector3& p3) {
        return approx_equal((p2 - p1).cross(p3 - p1).norm(), 0);
    }

    template <typename T>
    void sort3(T& a, T& b, T& c) {
        if (a > b) {
            std::swap(a, b);
        }
        if (b > c) {
            std::swap(b, c);
        }
        if (a > b) {
            std::swap(a, b);
        }
    }

} // namespace pe_phys_fracture