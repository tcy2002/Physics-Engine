#include "cloth/object/cloth_object.h"
#include <sstream>

namespace pe_phys_object {

    std::atomic<uint32_t> ClothObject::_global_id_counter(0);

    ClothObject::ClothObject(const std::string& name, pe::Real stiffness, pe::Real bending)
        : _name(name), _stiffness(stiffness), _bending(bending) {
        _global_id = _global_id_counter++;
    }

    void ClothObject::loadFromObj(const std::string& filename, const pe::Vector3& size) {
        std::fstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file." << filename << std::endl;
            return;
        }

        char buf[1024];
        while (file.getline(buf, 1024)) {
            std::stringstream ss(buf);
            std::string str;
            ss >> str;
            if (str == "v") {
                pe::Real x, y, z;
                ss >> x >> y >> z;
                _verts.push_back({x * size.x(), y * size.x(), z * size.x()});
            }
            else if (str == "f") {
                std::string vert;
                while (ss >> vert) {
                    int vi = std::atoi(vert.substr(0, vert.find_first_of('/')).c_str());
                    _tris.push_back(vi - 1);
                }
            }
        }

        rearangeMesh();
        rebuildMeshNormals();

        init = false;
        file.close();
    }

    void ClothObject::rearangeMesh() {
        // merge duplicate vertices
        std::map<int, int> vert_idx_map;
        std::unordered_map<pe::Vector3, int, pe::Vector3Hash, pe::Vector3Equal> vert_hash;
        pe::Array<pe::Vector3> new_verts;
        pe::Array<int> new_tris;

        // vertices
        for (int i = 0; i < _verts.size(); i++) {
            const pe::Vector3& v = _verts[i];
            auto it = vert_hash.find(v);
            if (it == vert_hash.end()) {
                vert_hash.insert({ v, new_verts.size() });
                vert_idx_map.insert({ i, new_verts.size() });
                new_verts.push_back(v);
            } else {
                vert_idx_map.insert({ i, it->second });
            }
        }

        // triangles
        for (int i = 0; i < _tris.size(); i += 3) {
            if (vert_idx_map[_tris[i]] == vert_idx_map[_tris[i + 1]] || 
                vert_idx_map[_tris[i + 1]] == vert_idx_map[_tris[i + 2]] || 
                vert_idx_map[_tris[i + 2]] == vert_idx_map[_tris[i]]) {
                continue; // skip degenerate triangles !!tricky
            }
            new_tris.push_back(vert_idx_map[_tris[i]]);
            new_tris.push_back(vert_idx_map[_tris[i + (_invert_order ? 2 : 1)]]);
            new_tris.push_back(vert_idx_map[_tris[i + (_invert_order ? 1 : 2)]]);
        }

        _verts = new_verts;
        _tris = new_tris;

        // vertex-triangle mapping
        _vert2tris.resize(_verts.size(), std::vector<int>());
        for (int i = 0; i < _tris.size(); i += 3) {
            for (int j = 0; j < 3; j++) {
                _vert2tris[_tris[i + j]].push_back(i / 3);
            }
        }
    }

    void ClothObject::rebuildMeshNormals() {
        if (_verts.size() == 0 || _tris.size() == 0) {
            return;
        }

        // per-face normal
        std::vector<pe::Vector3> face_nors(_tris.size() / 3);
        for (int i = 0; i < _tris.size(); i += 3) {
            pe::Vector3 v0 = _verts[_tris[i]];
            pe::Vector3 v1 = _verts[_tris[i + 1]];
            pe::Vector3 v2 = _verts[_tris[i + 2]];
            face_nors[i / 3] = -(v1 - v0).cross(v2 - v0).normalized();
        }

        // per-vertex normal
        _nors.assign(_verts.size(), pe::Vector3::Zero());
        for (int i = 0; i < _verts.size(); i++) {
            pe::Vector3 n = pe::Vector3::Zero();
            for (int j : _vert2tris[i]) {
                n += face_nors[j];
            }
            n.normalize();
            _nors[i] = n;
        }
    }

} // namespace pe_phys_object