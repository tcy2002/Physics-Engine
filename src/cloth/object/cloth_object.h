#pragma once

#include "rigid/phys_general.h"

namespace pe_phys_object {

    class ClothObject {
        COMMON_MEMBER_SET_GET(std::string, name, Name)
        COMMON_MEMBER_SET_GET(std::string, tag, Tag)
        COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)
        COMMON_MEMBER_SET_GET(pe::Real, stiffness, Stiffness)
        COMMON_MEMBER_SET_GET(pe::Real, bending, Bending)

    protected:
        bool init = false;
        pe::Array<pe::Vector3> _verts;
        pe::Array<int> _tris;
        pe::Array<pe::Vector3> _nors;
        bool _invert_order = false;

        // structure of pbd mesh
        pe::Array<pe::Array<int>> _vert2tris;
        void rearangeMesh();
        void rebuildMeshNormals();

    public:
        PE_API ClothObject(const std::string& name = "ClothObject", pe::Real stiffness = 1.0, pe::Real bending = 0.1);
        virtual ~ClothObject() {}
        PE_API void loadFromObj(const std::string& filename, const pe::Vector3& size = {1.0, 1.0, 1.0});

        virtual void step(pe::Real dt) = 0;

        const pe::Array<pe::Vector3>& getVertices() const { return _verts; }
        const pe::Array<int>& getTriangles() const { return _tris; }
        const pe::Array<pe::Vector3>& getNormals() const { return _nors; }

    protected:
        static std::atomic<uint32_t> _global_id_counter;
    };

} // namespace pe_phys_object