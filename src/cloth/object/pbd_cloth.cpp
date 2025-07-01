#include "pbd_cloth.h"

namespace pe_phys_object {

    PBDCloth::PBDCloth(const std::string& name, pe::Real stiffness, pe::Real bending)
        : ClothObject(name, stiffness, bending), 
          _damping(0.01), _mass(1.0),
          _thickness(0.01),
          _enable_self_collision(false), _self_collision_distance(4.0),
          _enable_bending(false), _accuracy(0.01), _max_iterations(10), _eps(1e-6),
          _gravity(pe::Vector3(0, -9.81, 0)), _boundaryAA(pe::Vector3(-200, -200, -200)),
          _boundaryBB(pe::Vector3(200, 200, 200)), _fixed_plane(0) {
    }

    void PBDCloth::step(pe::Real dt) {
        if (!init) {
            init = true;
            readMeshStructure();
        }

        // predict velocity and position using explicit euler
        predictVelocityPosition(dt);

        // project position using Gauss-Seidel iteration
        projectPosition(dt);

        rebuildMeshNormals();
    }

    void PBDCloth::readMeshStructure() {
        if (_verts.size() == 0 || _tris.size() == 0) {
            return;
        }

        // fixed vertices
        for (int i = 0; i < _verts.size(); i++) {
            if (_verts[i].y() > _fixed_plane) {
                _fixed_verts.insert(i);
            }
        }

        // mass: refer to the area of each triangle
        _inv_mass.resize(_verts.size());
        double total_area = 0;
        for (int i = 0; i < _tris.size(); i += 3) {
            pe::Vector3 v0 = _verts[_tris[i]];
            pe::Vector3 v1 = _verts[_tris[i + 1]];
            pe::Vector3 v2 = _verts[_tris[i + 2]];
            pe::Vector3 n = (v1 - v0).cross(v2 - v0);
            double area = n.norm() / 2.0;
            _inv_mass[_tris[i]] += area / 3.0;
            _inv_mass[_tris[i + 1]] += area / 3.0;
            _inv_mass[_tris[i + 2]] += area / 3.0;
            total_area += area;
        }
        for (size_t i = 0; i < _inv_mass.size(); i++) {
            _inv_mass[i] = total_area / (_inv_mass[i] * _mass);
        }
        for (size_t i = 0; i < _inv_mass.size(); i++) {
            if (_fixed_verts.find(i) != _fixed_verts.end()) {
                _inv_mass[i] = 0; // fixed vertex has infinite mass
            }
        }

        _verts_orig = _verts;
        _verts_new = _verts;
        _vels.resize(_verts.size(), pe::Vector3::Zero());
        _tri_should_update_hash.resize(_tris.size(), true);

        // structure
        _max_tri_size = 0;
        _min_tri_size = 1e10;
        for (int i = 0; i < _tris.size(); i += 3) {
            for (int j = 0; j < 3; j++) {
                int v0 = _tris[i + j];
                int v1 = _tris[i + (j + 1) % 3];
                int v2 = _tris[i + (j + 2) % 3];
                if (v0 > v1) std::swap(v0, v1);
                if (_edges.find({ v0, v1 }) == _edges.end()) {
                    double dist = (_verts[v0]- _verts[v1]).norm();
                    _edges.insert({ {v0, v1}, dist });
                    _max_tri_size = PE_MAX(_max_tri_size, dist);
                    _min_tri_size = PE_MIN(_min_tri_size, dist);
                }
                _edge2verts[{v0, v1}].push_back(v2);
            }
        }

        // bending angle
        if (_enable_bending) {
            for (auto& ev : _edge2verts) {
                if (ev.second.size() != 2) continue;
                const pe::Vector3& v0 = _verts[ev.first.first];
                const pe::Vector3& v1 = _verts[ev.first.second];
                const pe::Vector3& v2 = _verts[ev.second[0]];
                const pe::Vector3& v3 = _verts[ev.second[1]];
                pe::Vector3 n1 = (v1 - v0).cross(v2 - v0).normalized();
                pe::Vector3 n2 = (v1 - v0).cross(v3 - v0).normalized();
                int sign = n1.cross(n2).dot(v1 - v0) > 0 ? 1 : -1;
                double angle = PE_ACOS(n1.dot(n2));
                _edge_angle[ev.first] = std::make_pair(angle, sign);
            }
        }
        
        _avg_tri_size = 0;
        for (auto& edge : _edges) {
            _avg_tri_size += edge.second;
        }
        _avg_tri_size /= _edges.size();

        // self collision ignore triangles
        _vert_ignore_tris.resize(_verts.size(), std::set<int>());
        for (int i = 0; i < _tris.size(); i += 3) {
            for (int j = 0; j < 3; j++) {
                int v0 = _tris[i + j];
                int v1 = _tris[i + (j + 1) % 3];
                _vert_ignore_tris[v0].insert(i / 3);
                _vert_ignore_tris[v1].insert(i / 3);
            }
        }
        for (int i = 0; i < _verts.size(); i++) {
            for (int j = 0; j < _tris.size(); j += 3) {
                const pe::Vector3& p = _verts[i];
                const pe::Vector3& v0 = _verts[_tris[j]];
                const pe::Vector3& v1 = _verts[_tris[j + 1]];
                const pe::Vector3& v2 = _verts[_tris[j + 2]];
                pe::Vector3 center = (v0 + v1 + v2) / 3;
                if ((center - p).squaredNorm() < _self_collision_distance * _self_collision_distance) {
                    _vert_ignore_tris[i].insert(j / 3);
                }
            }
        }
    }

    pe::Real PBDCloth::project2Boundary(pe::Vector3& v) {
        pe::Real residual = 0;
            if (v.x() < _boundaryAA.x()) {
            residual += (_boundaryAA.x() - v.x()) * (_boundaryAA.x() - v.x());
            v.x() = _boundaryAA.x();
        }
        else if (v.x() > _boundaryBB.x()) {
            residual += (v.x() - _boundaryBB.x()) * (v.x() - _boundaryBB.x());
            v.x() = _boundaryBB.x();
        }
        if (v.y() < _boundaryAA.y()) {
            residual += (_boundaryAA.y() - v.y()) * (_boundaryAA.y() - v.y());
            v.y() = _boundaryAA.y();
        }
        else if (v.y() > _boundaryBB.y()) {
            residual += (v.y() - _boundaryBB.y()) * (v.y() - _boundaryBB.y());
            v.y() = _boundaryBB.y();
        }
        if (v.z() < _boundaryAA.z()) {
            residual += (_boundaryAA.z() - v.z()) * (_boundaryAA.z() - v.z());
            v.z() = _boundaryAA.z();
        }
        else if (v.z() > _boundaryBB.z()) {
            residual += (v.z() - _boundaryBB.z()) * (v.z() - _boundaryBB.z());
            v.z() = _boundaryBB.z();
        }
        return residual;
    }

    uint32_t PBDCloth::getVertHashFromPos(const pe::Vector3 v) {
        int grid = (int)_avg_tri_size + 1;
        return int(v.x() - _boundaryAA.x()) / grid * 1048576 + int(v.y() - _boundaryAA.y()) / grid * 1024 + int(v.z() - _boundaryAA.z()) / grid;
    }

    uint32_t PBDCloth::getVertHashFromGrid(const int vx, const int vy, const int vz) {
        return vx * 1048576 + vy * 1024 + vz;
    }

    void PBDCloth::getVertGridFromPos(const pe::Vector3& v, int& vx, int& vy, int& vz) {
        int grid = (int)_avg_tri_size + 1;
        vx = (int)(v.x() - _boundaryAA.x()) / grid;
        vy = (int)(v.y() - _boundaryAA.y()) / grid;
        vz = (int)(v.z() - _boundaryAA.z()) / grid;
    }

    void PBDCloth::getVertGridFromHash(uint32_t hash, int& vx, int& vy, int& vz) {
        vx = hash / 1048576;
        vy = (hash % 1048576) / 1024;
        vz = hash % 1024;
    }

    void PBDCloth::updateTriangleHash(int tri) {
        pe::Vector3& p0 = _verts[_tris[tri * 3]];
        pe::Vector3& p1 = _verts[_tris[tri * 3 + 1]];
        pe::Vector3& p2 = _verts[_tris[tri * 3 + 2]];
        pe::Vector3 posMin = pe::Vector3(PE_MIN3(p0.x(), p1.x(), p2.x()), PE_MIN3(p0.y(), p1.y(), p2.y()), PE_MIN3(p0.z(), p1.z(), p2.z()));
        pe::Vector3 posMax = pe::Vector3(PE_MAX3(p0.x(), p1.x(), p2.x()), PE_MAX3(p0.y(), p1.y(), p2.y()), PE_MAX3(p0.z(), p1.z(), p2.z()));
        int gridMinX, gridMinY, gridMinZ;
        getVertGridFromPos(posMin, gridMinX, gridMinY, gridMinZ);
        int gridMaxX, gridMaxY, gridMaxZ;
        getVertGridFromPos(posMax, gridMaxX, gridMaxY, gridMaxZ);

        // remove old values
        auto range_tri = _tri2hash.equal_range(tri);
        for (auto it = range_tri.first; it != range_tri.second; ++it) {
            auto range_hash = _hash2tri.equal_range(it->second);
            for (auto it2 = range_hash.first; it2 != range_hash.second; ++it2) {
                if (it2->second == tri) {
                    _hash2tri.erase(it2);
                    break;
                }
            }
        }
        _tri2hash.erase(range_tri.first, range_tri.second);

        // insert new values
        for (int x = gridMinX; x <= gridMaxX; x++) {
            for (int y = gridMinY; y <= gridMaxY; y++) {
                for (int z = gridMinZ; z <= gridMaxZ; z++) {
                    uint32_t hash = getVertHashFromGrid(x, y, z);
                    _hash2tri.insert({ hash, tri });
                    _tri2hash.insert({ tri, hash });
                }
            }
        }
    }

    void PBDCloth::updateTriangleHash(const pe::Array<pe::Vector3>& last_verts, const pe::Array<pe::Vector3>& new_verts) {
        for (int i = 0; i < new_verts.size(); i++) {
            if (getVertHashFromPos(new_verts[i]) != getVertHashFromPos(last_verts[i])) {
                for (auto tri : _vert2tris[i]) {
                    _tri_should_update_hash[tri] = true;
                }
            }
        }
        for (int i = 0; i < _tris.size() / 3; i++) {
            if (_tri_should_update_hash[i]) {
                updateTriangleHash(i);
                _tri_should_update_hash[i] = false;
            }
        }
    }

    void PBDCloth::predictVelocityPosition(pe::Real dt) {
        if (_verts.size() == 0 || _vels.size() == 0) {
            return;
        }

        // explicit euler
        for (int i = 0; i < _verts.size(); i++) {
            if (_fixed_verts.find(i) != _fixed_verts.end()) {
                continue;
            }

            const pe::Vector3& p = _verts[i];
            pe::Vector3& v = _vels[i];
            v += _gravity * dt;

            // velocity damping
            //v *= PE_MAX(1 - _damping / m_mass[i] * dt, 0);

            _verts_new[i] = p + v * dt;
        }
    }

    static bool isVertexInsideTriangle(const pe::Vector3& p, const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2, double eps) {
        // Determine if point p is inside the triangle prism formed by v0, v1, and v2
        pe::Vector3 n = (v1 - v0).cross(v2 - v0).normalized();
        return (p - v0).dot((v1 - v0).cross(n)) < 0 &&
            (p - v1).dot((v2 - v1).cross(n)) < 0 &&
            (p - v2).dot((v0 - v2).cross(n)) < 0;
    }

    void PBDCloth::projectPosition(pe::Real dt) {
        if (_edges.size() == 0) {
            return;
        }

        // gauss-seidel iteration
        double residual = _accuracy + 1;
        int it;
        pe::Array<pe::Vector3> lastVertsNew = _verts;
        for (it = 0; it < _max_iterations && residual > _accuracy; it++) {
            residual = 0;

            // structure
            for (auto& edge_s : _edges) {
                int v0 = edge_s.first.first;
                int v1 = edge_s.first.second;
                if (_fixed_verts.find(v0) != _fixed_verts.end() && _fixed_verts.find(v1) != _fixed_verts.end()) {
                    continue;
                }

                pe::Vector3& p0 = _verts_new[v0];
                pe::Vector3& p1 = _verts_new[v1];
                pe::Vector3 d = p1 - p0;
                double l0 = edge_s.second;
                double l1 = d.norm();
                if (l1 > _eps) {
                    pe::Vector3 dir = d / l1;
                    double diff = l1 - l0;
                    pe::Vector3 diffV = dir * diff * _stiffness;
                    double mass_ratio = _inv_mass[v0] / (_inv_mass[v0] + _inv_mass[v1]);
                    p0 += diffV * mass_ratio;
                    p1 -= diffV * (1 - mass_ratio);
                    residual += diff * diff;
                }
            }

            // bending (angles)
            if (_enable_bending) {
                for (auto& ev : _edge2verts) {
                    if (ev.second.size() != 2) continue;
                    pe::Vector3& p1 = _verts_new[ev.first.first];
                    pe::Vector3& p2 = _verts_new[ev.first.second];
                    pe::Vector3& p3 = _verts_new[ev.second[0]];
                    pe::Vector3& p4 = _verts_new[ev.second[1]];

                    /*int v1y = FMath::RoundToInt(p1.y() + 121.5) / 2;
                    int v1x = FMath::RoundToInt(p1.x() + v1y) / 2;
                    int v1i = (1 + v1y) * v1y / 2 + v1x;
                    int v2y = FMath::RoundToInt(p2.y() + 121.5) / 2;
                    int v2x = FMath::RoundToInt(p2.x() + v2y) / 2;
                    int v2i = (1 + v2y) * v2y / 2 + v2x;
                    int v3y = FMath::RoundToInt(p3.y() + 121.5) / 2;
                    int v3x = FMath::RoundToInt(p3.x() + v3y) / 2;
                    int v3i = (1 + v3y) * v3y / 2 + v3x;
                    int v4y = FMath::RoundToInt(p4.y() + 121.5) / 2;
                    int v4x = FMath::RoundToInt(p4.x() + v4y) / 2;
                    int v4i = (1 + v4y) * v4y / 2 + v4x;*/

                    pe::Vector3 p2_p1 = p2 - p1;
                    pe::Vector3 p3_p1 = p3 - p1;
                    pe::Vector3 p4_p1 = p4 - p1;
                    pe::Vector3 n1 = p2_p1.cross(p3_p1).normalized();
                    pe::Vector3 n2 = p2_p1.cross(p4_p1).normalized();
                    double d = n1.dot(n2);
                    double angle = PE_ACOS(d);
                    int sign = (n1.cross(n2).dot(p2_p1) > 0 ? 1 : -1) * _edge_angle[ev.first].second;
                    constexpr double pi2 = PE_PI * 2.0;
                    //if (sign < 0) angle = pi2 - angle;
                    double diff = angle - _edge_angle[ev.first].first;
                    /*if (PE_ABS(diff) > 0.01) {
                        UE_LOG(LogTemp, Log, TEXT("ori ang: %f, cur ang: %f, sign: %d"), _edge_angle[ev.first].first, angle, sign);
                        pe::Vector3& p1o = _verts_orig[ev.first.first];
                        pe::Vector3& p2o = _verts_orig[ev.first.second];
                        pe::Vector3& p3o = _verts_orig[ev.second[0]];
                        pe::Vector3& p4o = _verts_orig[ev.second[1]];
                        UE_LOG(LogTemp, Log, TEXT("ori p1: %s, p2: %s, p3: %s, p4: %s"), *p1o.ToString(), *p2o.ToString(), *p3o.ToString(), *p4o.ToString());
                        UE_LOG(LogTemp, Log, TEXT("cur p1: %s, p2: %s, p3: %s, p4: %s"), *p1.ToString(), *p2.ToString(), *p3.ToString(), *p4.ToString());
                    }*/
                    double p2_c_p3 = p2_p1.cross(p3_p1).norm();
                    double p2_c_p4 = p2_p1.cross(p4_p1).norm();
                    if (p2_c_p3 > _eps && p2_c_p4 > _eps) {
                        pe::Vector3 q2 = -(p3_p1.cross(n2) + d * n1.cross(p3_p1)) / p2_c_p3 - (p4_p1.cross(n1) + d * n2.cross(p4_p1)) / p2_c_p4;
                        pe::Vector3 q3 = (p2_p1.cross(n2) + d * n1.cross(p2_p1)) / p2_c_p3;
                        pe::Vector3 q4 = (p2_p1.cross(n1) + d * n2.cross(p2_p1)) / p2_c_p4;
                        pe::Vector3 q1 = -(q2 + q3 + q4);
                        double sqrt_1_d2 = PE_SQRT(PE_MAX(0, 1 - d * d));
                        double w1 = _inv_mass[ev.first.first];
                        double w2 = _inv_mass[ev.first.second];
                        double w3 = _inv_mass[ev.second[0]];
                        double w4 = _inv_mass[ev.second[1]];
                        double sum_wq = w1 * q1.squaredNorm() + w2 * q2.squaredNorm() + w3 * q3.squaredNorm() + w4 * q4.squaredNorm();
                        double s = sqrt_1_d2 * diff / sum_wq * _bending;
                        if (!std::isnan(s)) {
                            pe::Vector3 dp1 = w1 * s * q1;
                            pe::Vector3 dp2 = w2 * s * q2;
                            pe::Vector3 dp3 = w3 * s * q3;
                            pe::Vector3 dp4 = w4 * s * q4;
                            /*UE_LOG(LogTemp, Log, TEXT("dp1: %s, dp2: %s, dp3: %s, dp4: %s"), *dp1.ToString(), *dp2.ToString(), *dp3.ToString(), *dp4.ToString());
                            UE_LOG(LogTemp, Log, TEXT("p1: %d, p2: %d, p3: %d, p4: %d"), v1i, v2i, v3i, v4i);
                            UE_LOG(LogTemp, Log, TEXT("w1: %f, w1: %f, w1: %f, w1: %f"), w1, w2, w3, w4);
                            UE_LOG(LogTemp, Log, TEXT("diff: %f, sum_wq: %f, sqrt_l_d2: %f"), diff, sum_wq, sqrt_1_d2);*/
                            p1 -= dp1;
                            p2 -= dp2;
                            p3 -= dp3;
                            p4 -= dp4;
                            residual += diff * diff;
                        }
                    }
                }
            }

            // self collision
            if (_enable_self_collision) {
                // update triangle hash
                updateTriangleHash(lastVertsNew, _verts_new);
                lastVertsNew = _verts_new;

                // find self-collision pairs
                _self_collision_pairs.clear();
                for (int i = 0; i < _verts.size(); i++) {
                    findSelfCollisionPairs(i);
                }

                // handle self-collision pairs
                for (auto& pair : _self_collision_pairs) {
                    int vert = pair.first;
                    int tri = pair.second;
                    pe::Vector3& p = _verts_new[vert];
                    pe::Vector3& p0 = _verts_new[_tris[tri * 3]];
                    pe::Vector3& p1 = _verts_new[_tris[tri * 3 + 1]];
                    pe::Vector3& p2 = _verts_new[_tris[tri * 3 + 2]];

                    const pe::Vector3& p_last = _verts[vert];
                    const pe::Vector3& p0_last = _verts[_tris[tri * 3]];
                    const pe::Vector3& p1_last = _verts[_tris[tri * 3 + 1]];
                    const pe::Vector3& p2_last = _verts[_tris[tri * 3 + 2]];
                    pe::Vector3 n = (p1 - p0).cross(p2 - p0).normalized();
                    double dist = PE_ABS(n.dot(p - p0));
                    pe::Vector3 n_last = (p1_last - p0_last).cross(p2_last - p0_last).normalized();
                    double dist_last = PE_ABS(n_last.dot(p_last - p0_last));
                    double mass_ratio = _inv_mass[vert] / (_inv_mass[vert] + _inv_mass[_tris[tri * 3]] + _inv_mass[_tris[tri * 3 + 1]] + _inv_mass[_tris[tri * 3 + 2]]);

                    // collision happens under 2 conditions: 
                    //// the vertex moves through the triangle
                    //// or dist of the vertex to the triangle is smaller than the thickness
                    if (!std::isnan(mass_ratio) && (dist * dist_last < 0 || PE_ABS(dist) < _thickness)) {
                        double d = (dist_last < 0 ? _thickness : -_thickness);
                        //UE_LOG(LogTemp, Error, TEXT("Self collision before. dist: %f, n: %s, d: %f, mass_ratio: %f, p: %s, p0: %s, p1: %s, p2: %s"), dist, *n.ToString(), d, mass_ratio, *p.ToString(), *p0.ToString(), *p1.ToString(), *p2.ToString());
                        p -= n * (dist + d) * mass_ratio;
                        pe::Vector3 dp = n * (dist + d) * (1 - mass_ratio);
                        p0 += dp;
                        p1 += dp;
                        p2 += dp;
                        residual += (dist + d) * (dist + d);
                        //UE_LOG(LogTemp, Error, TEXT("Self collision after. dist: %f, n: %s, p: %s, p0: %s, p1: %s, p2: %s"), dist, *n.ToString(), *p.ToString(), *p0.ToString(), *p1.ToString(), *p2.ToString());
                    }
                }
            }

            // boundary
            for (auto& v : _verts_new) {
                residual += project2Boundary(v);
            }

            residual = PE_SQRT(residual);
            residual /= _edges.size();
        }
        

        if (_enable_self_collision) {
            updateTriangleHash(lastVertsNew, _verts_new);
        }

        // recalculate velocity
        for (int i = 0; i < _verts.size(); i++) {
            pe::Vector3& p = _verts[i];
            const pe::Vector3& p_new = _verts_new[i];
            pe::Vector3& v = _vels[i];
            v = (p_new - p) / dt;
            p = p_new;
        }
    }

    void PBDCloth::findSelfCollisionPairs(int vert) {
        auto range_tri = _hash2tri.equal_range(getVertHashFromPos(_verts[vert]));
        for (auto it = range_tri.first; it != range_tri.second; ++it) {
            int tri = it->second;
            if (_vert_ignore_tris[vert].find(tri) != _vert_ignore_tris[vert].end()) {
                continue;
            }
            pe::Vector3& p = _verts_new[vert];
            pe::Vector3& p0 = _verts_new[_tris[tri * 3]];
            pe::Vector3& p1 = _verts_new[_tris[tri * 3 + 1]];
            pe::Vector3& p2 = _verts_new[_tris[tri * 3 + 2]];
            if (!isVertexInsideTriangle(p, p0, p1, p2, _eps)) {
                continue;
            }
            _self_collision_pairs.push_back({ vert, tri });
        } 
    }

} // namespace pe_phys_object