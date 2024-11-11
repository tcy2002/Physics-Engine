#include "simulator.h"
#include "json/json.hpp"
#include "opts/cxxopts.hpp"
#include "utils/file_system.h"
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <phys/shape/default_mesh.h>

#include "gltf/tiny_gltf.h"

namespace pe_intf {
    static bool isNodeTypeEqual(nlohmann::json::value_t type, nlohmann::json::value_t target) {
        static std::unordered_map<nlohmann::json::value_t, int> type_map = {
            {nlohmann::json::value_t::null, 0},
            {nlohmann::json::value_t::object, 1},
            {nlohmann::json::value_t::array, 2},
            {nlohmann::json::value_t::string, 3},
            {nlohmann::json::value_t::boolean, 4},
            {nlohmann::json::value_t::number_integer, 5},
            {nlohmann::json::value_t::number_unsigned, 5},
            {nlohmann::json::value_t::number_float, 5}
        };
        return type_map[type] == type_map[target];
    }

    static bool jsonNodeExists(const nlohmann::json& node, const std::string& name, nlohmann::json::value_t type) {
        static std::unordered_map<nlohmann::json::value_t, std::string> type_str = {
            {nlohmann::json::value_t::null, "null"},
            {nlohmann::json::value_t::object, "object"},
            {nlohmann::json::value_t::array, "array"},
            {nlohmann::json::value_t::string, "string"},
            {nlohmann::json::value_t::boolean, "boolean"},
            {nlohmann::json::value_t::number_integer, "number (integer or float)"},
            {nlohmann::json::value_t::number_unsigned, "number (integer or float)"},
            {nlohmann::json::value_t::number_float, "number (integer or float)"}
        };
        if (node.find(name) == node.end() || node[name].is_null()) {
            return false;
        }
        if (!isNodeTypeEqual(node[name].type(), type)) {
            PE_LOG_CUSTOM_ERROR << "Node " << name << " should be a " << type_str[type] << "." << PE_CUSTOM_ENDL;
            throw std::runtime_error("Node type error.");
        }
        return true;
    }

    static bool jsonArrayExists(const nlohmann::json& node, const std::string& name, int size, nlohmann::json::value_t item_type) {
        static std::unordered_map<nlohmann::json::value_t, std::string> type_str = {
            {nlohmann::json::value_t::null, "null"},
            {nlohmann::json::value_t::object, "object"},
            {nlohmann::json::value_t::array, "array"},
            {nlohmann::json::value_t::string, "string"},
            {nlohmann::json::value_t::boolean, "boolean"},
            {nlohmann::json::value_t::number_integer, "number_integer"},
            {nlohmann::json::value_t::number_unsigned, "number_unsigned"},
            {nlohmann::json::value_t::number_float, "number_float"}
        };
        if (node.find(name) == node.end() || node[name].is_null()) {
            return false;
        }
        if (node[name].type() != nlohmann::json::value_t::array) {
            PE_LOG_CUSTOM_ERROR << "Node " << name << " should be an array." << PE_CUSTOM_ENDL;
            throw std::runtime_error("Node type error.");
        }
        if (size != 0 && node[name].size() != size) {
            PE_LOG_CUSTOM_ERROR << "Node " << name << " should have " << size << " elements." << PE_CUSTOM_ENDL;
            throw std::runtime_error("Node size error.");
        }
        for (auto& item : node[name]) {
            if (!isNodeTypeEqual(item.type(), item_type)) {
                PE_LOG_CUSTOM_ERROR << "Node " << name << " should have elements of type " << type_str[item_type] << "." << PE_CUSTOM_ENDL;
                throw std::runtime_error("Node item type error.");
            }
        }
        return true;
    }

    void Simulator::saveScene(const std::string& path_to_write) {
        std::string json_file;
        if (path_to_write.empty()) {
            json_file = utils::FileSystem::fileDialog(1, path_to_write, "Scene Files", "json");
        } else {
            json_file = path_to_write;
            if (!utils::FileSystem::fileExists(json_file)) {
                auto dir = utils::FileSystem::getFilePath(json_file);
                std::cout << utils::FileSystem::getFilePath("..\\..\\SampleScenes\\data\\1.json") << std::endl;
                if (!utils::FileSystem::isDirectory(dir)) {
                    utils::FileSystem::makeDir(dir);
                }
            }
        }
        if (json_file.empty()) {
            PE_LOG_CUSTOM_ERROR << "No scene file specified." << PE_CUSTOM_ENDL;
            return;
        }

        PE_LOG_CUSTOM_INFO << "Save scene to: " << json_file << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // save configuration
        //////////////////////////////////////////////////////////////////////////
        nlohmann::json data;
        data["Configuration"]["gravity"] = {_world.getGravity().x, _world.getGravity().y, _world.getGravity().z};
        data["Configuration"]["sleep_linear_velocity2_threshold"] = _world.getSleepLinVel2Threshold();
        data["Configuration"]["sleep_angular_velocity2_threshold"] = _world.getSleepAngVel2Threshold();
        data["Configuration"]["sleep_time_threshold"] = _world.getSleepTimeThreshold();
        PE_LOG_CUSTOM_INFO << "Saved configuration." << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // save rigidbodies
        //////////////////////////////////////////////////////////////////////////
        auto& rbs = data["RigidBodies"];
        for (auto rb : _world.getRigidBodies()) {
            nlohmann::json rb_data;
            rb_data["name"] = rb->getName();
            rb_data["tag"] = rb->getTag();
            rb_data["ignore_collision"] = rb->isIgnoreCollision();
            rb_data["kinematic"] = rb->isKinematic();
            if (!rb->isKinematic()) {
                rb_data["linear_damping"] = rb->getLinearDamping();
                rb_data["angular_damping"] = rb->getAngularDamping();
                rb_data["linear_velocity"] = {rb->getLinearVelocity().x, rb->getLinearVelocity().y, rb->getLinearVelocity().z};
                rb_data["angular_velocity"] = {rb->getAngularVelocity().x, rb->getAngularVelocity().y, rb->getAngularVelocity().z};
            }
            rb_data["lifetime"] = rb->getLifeTime();
            rb_data["fracturable"] = rb->isFracturable();
            if (rb->isFracturable()) {
                rb_data["fracture_threshold"] = static_cast<pe_phys_object::FracturableObject*>(rb)->getThreshold();
            }
            auto transform = rb->getTransform();
            rb_data["position"] = {transform.getOrigin().x, transform.getOrigin().y, transform.getOrigin().z};
            auto rot = transform.getBasis();
            rb_data["rotation"] = {rot[0][0], rot[0][1], rot[0][2],
                                     rot[1][0], rot[1][1], rot[1][2],
                                     rot[2][0], rot[2][1], rot[2][2]};
            rb_data["mass"] = rb->getMass();
            rb_data["friction"] = rb->getFrictionCoeff();
            rb_data["restitution"] = rb->getRestitutionCoeff();
            auto shape = rb->getCollisionShape();
            switch (shape->getType()) {
                case pe_phys_shape::ShapeType::Box: {
                    rb_data["type"] = "box";
                    auto size = static_cast<pe_phys_shape::BoxShape*>(shape)->getSize();
                    rb_data["scale"] = {size.x, size.y, size.z};
                    break;
                }
                case pe_phys_shape::ShapeType::Sphere: {
                    rb_data["type"] = "sphere";
                    auto radius = static_cast<pe_phys_shape::SphereShape*>(shape)->getRadius();
                    rb_data["scale"] = {radius};
                    break;
                }
                case pe_phys_shape::ShapeType::Cylinder: {
                    rb_data["type"] = "cylinder";
                    auto radius = static_cast<pe_phys_shape::CylinderShape*>(shape)->getRadius();
                    auto height = static_cast<pe_phys_shape::CylinderShape*>(shape)->getHeight();
                    rb_data["scale"] = {radius, height};
                    break;
                }
                case pe_phys_shape::ShapeType::ConvexMesh: {
                    rb_data["type"] = "convex";
                    rb_data["mesh"] = ((pe_phys_shape::ConvexMeshShape*)shape)->getMeshPath(); // user should change this in the json file
                    pe::Vector3 scale = ((pe_phys_shape::ConvexMeshShape*)shape)->getScale();
                    rb_data["scale"] = {scale.x, scale.y, scale.z}; // user should change this in the json file
                    break;
                }
                case pe_phys_shape::ShapeType::ConcaveMesh: {
                    rb_data["type"] = "concave";
                    rb_data["mesh"] = ((pe_phys_shape::ConcaveMeshShape*)shape)->getMeshPath(); // user should change this in the json file
                    pe::Vector3 scale = ((pe_phys_shape::ConcaveMeshShape*)shape)->getScale();
                    rb_data["scale"] = {scale.x, scale.y, scale.z}; // user should change this in the json file
                    break;
                }
                case pe_phys_shape::ShapeType::Compound: {
                    rb_data["type"] = "compound";
                    auto compound = static_cast<pe_phys_shape::CompoundShape*>(shape);
                    auto& shapes = compound->getShapes();
                    auto& shapes_data = rb_data["shapes"];
                    for (auto& s : shapes) {
                        nlohmann::json shape_data;
                        shape_data["mass_ratio"] = s.mass_ratio;
                        shape_data["position"] = {s.local_transform.getOrigin().x, s.local_transform.getOrigin().y, s.local_transform.getOrigin().z};
                        auto rot = s.local_transform.getBasis();
                        shape_data["rotation"] = {rot[0][0], rot[0][1], rot[0][2],
                                                  rot[1][0], rot[1][1], rot[1][2],
                                                  rot[2][0], rot[2][1], rot[2][2]};
                        switch (s.shape->getType()) {
                            case pe_phys_shape::ShapeType::Box: {
                                shape_data["type"] = "box";
                                auto size = static_cast<pe_phys_shape::BoxShape*>(s.shape)->getSize();
                                shape_data["scale"] = {size.x, size.y, size.z};
                                break;
                            }
                            case pe_phys_shape::ShapeType::Sphere: {
                                shape_data["type"] = "sphere";
                                auto radius = static_cast<pe_phys_shape::SphereShape*>(s.shape)->getRadius();
                                shape_data["scale"] = {radius};
                                break;
                            }
                            case pe_phys_shape::ShapeType::Cylinder: {
                                shape_data["type"] = "cylinder";
                                auto radius = static_cast<pe_phys_shape::CylinderShape*>(s.shape)->getRadius();
                                auto height = static_cast<pe_phys_shape::CylinderShape*>(s.shape)->getHeight();
                                shape_data["scale"] = {radius, height};
                                break;
                            }
                            case pe_phys_shape::ShapeType::ConvexMesh: {
                                shape_data["type"] = "convex";
                                shape_data["mesh"] = ((pe_phys_shape::ConvexMeshShape*)shape)->getMeshPath(); // user should change this in the json file
                                pe::Vector3 scale = ((pe_phys_shape::ConvexMeshShape*)shape)->getScale();
                                shape_data["scale"] = {scale.x, scale.y, scale.z}; // user should change this in the json file
                                break;
                            }
                            default:
                                break; // concave and compound shapes are not supported for compound sub-shapes
                        }
                        shapes_data.push_back(shape_data);
                    }
                }
            }
            rbs.push_back(rb_data);
        }
        PE_LOG_CUSTOM_INFO << "Saved " << rbs.size() << " rigidbodies." << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // save damage sources
        //////////////////////////////////////////////////////////////////////////
        auto& dss = data["DamageSources"];
        for (auto ds : _world.getFractureSources()) {
            nlohmann::json ds_data;
            ds_data["type"] = ds.type == pe_phys_fracture::FractureType::Sphere ? "sphere" : "cylinder";
            ds_data["position"] = {ds.position.x, ds.position.y, ds.position.z};
            ds_data["intensity"] = {ds.intensity.x, ds.intensity.y, ds.intensity.z};
            dss.push_back(ds_data);
        }
        PE_LOG_CUSTOM_INFO << "Saved " << dss.size() << " damage sources." << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // save constraints
        //////////////////////////////////////////////////////////////////////////
        // not implemented

        std::ofstream file(json_file);
        file << data.dump(4);
        file.close();
        PE_LOG_CUSTOM_INFO << "Scene saved." << PE_CUSTOM_ENDL;
    }

    bool Simulator::loadScene(int argc, char** argv) {
        cxxopts::Options options(argv[0], "RigidBody Dynamics - DALAB");
        options.add_options()
            ("h,help", "Print help")
            ("no-gui", "Disable GUI")
            ("l,load-scene", "Scene file to load", cxxopts::value<std::string>())
            ("s,save-to", "Path to save the scene file. When using this argument, downloading is automatically turned on", cxxopts::value<std::string>()->default_value("./data"))
            ("max-frame", "Max frame to simulate", cxxopts::value<int>()->default_value("2147483647"))
            ("target-framerate", "Target framerate", cxxopts::value<int>()->default_value("60"));
        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            PE_LOG_CUSTOM_INFO << options.help() << PE_CUSTOM_ENDL;
            return false;
        }
        if (result.count("no-gui")) {
            use_gui = !result["no-gui"].as<bool>();
        }
        if (result.count("max-frame")) {
            max_frame = result["max-frame"].as<int>();
        }
        if (result.count("target-framerate")) {
            target_framerate = result["target-framerate"].as<int>();
        }
        if (result.count("save-to")) {
            _saving = true;
            save_path = result["save-to"].as<std::string>();
        }

        std::string json_file;
        if (result.count("load-scene")) {
            json_file = result["load-scene"].as<std::string>();
        } else {
            PE_LOG_CUSTOM_INFO << "Please select a scene file." << PE_CUSTOM_ENDL;
            json_file = utils::FileSystem::fileDialog(0, PE_DATA_PATH, "Scene Files", "json");
        }

        if (json_file.empty()) {
            PE_LOG_CUSTOM_ERROR << "No scene file specified. Please select a json file in file dialog" << PE_CUSTOM_ENDL;
            return false;
        }

        PE_LOG_CUSTOM_INFO << "Load scene: " << json_file << PE_CUSTOM_ENDL;

        std::ifstream file(json_file);
        if (!file.is_open()) {
            PE_LOG_CUSTOM_ERROR << "Failed to open file: " << json_file << PE_CUSTOM_ENDL;
            return false;
        }

        nlohmann::json data;
        try {
            data = nlohmann::json::parse(file);
        } catch (const std::exception& e) {
            PE_LOG_CUSTOM_ERROR << "Failed to parse json file: " << e.what() << PE_CUSTOM_ENDL;
            return false;
        }

        //////////////////////////////////////////////////////////////////////////
        // read configuration
        //////////////////////////////////////////////////////////////////////////
        if (jsonNodeExists(data, "Configuration", nlohmann::json::value_t::object)) {
            auto& config = data["Configuration"];
            if (jsonArrayExists(config, "gravity", 3, nlohmann::json::value_t::number_float)) {
                _world.setGravity(pe::Vector3(config["gravity"][0], config["gravity"][1], config["gravity"][2]));
            }
            if (jsonNodeExists(config, "sleep_linear_velocity2_threshold", nlohmann::json::value_t::number_float)) {
                _world.setSleepLinVel2Threshold(config["sleep_linear_velocity2_threshold"]);
            }
            if (jsonNodeExists(config, "sleep_angular_velocity2_threshold", nlohmann::json::value_t::number_float)) {
                _world.setSleepAngVel2Threshold(config["sleep_angular_velocity2_threshold"]);
            }
            if (jsonNodeExists(config, "sleep_time_threshold", nlohmann::json::value_t::number_float)) {
                _world.setSleepTimeThreshold(config["sleep_time_threshold"]);
            }
        }
        PE_LOG_CUSTOM_INFO << "Loaded configuration." << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // read rigidbodies
        //////////////////////////////////////////////////////////////////////////
        int k = 0;
        if (jsonArrayExists(data, "RigidBodies", 0, nlohmann::json::value_t::object)) {
            auto& rbs = data["RigidBodies"];
            for (auto& rb : rbs) {
                pe_phys_object::RigidBody* rb_obj;
                if (jsonNodeExists(rb, "fracturable", nlohmann::json::value_t::boolean) && rb["fracturable"].get<bool>()) {
                    rb_obj = new pe_phys_object::FracturableObject();
                    if (jsonNodeExists(rb, "fracture_threshold", nlohmann::json::value_t::number_float))
                        static_cast<pe_phys_object::FracturableObject*>(rb_obj)->setThreshold(rb["fracture_threshold"]);
                } else {
                    rb_obj = new pe_phys_object::RigidBody();
                }
                if (jsonNodeExists(rb, "name", nlohmann::json::value_t::string)) rb_obj->setName(rb["name"]);
                if (jsonNodeExists(rb, "tag", nlohmann::json::value_t::string)) rb_obj->setTag(rb["tag"]);
                if (jsonNodeExists(rb, "ignore_collision", nlohmann::json::value_t::boolean)) rb_obj->setIgnoreCollision(rb["ignore_collision"]);
                if (jsonNodeExists(rb, "kinematic", nlohmann::json::value_t::boolean)) rb_obj->setKinematic(rb["kinematic"]);
                if (!rb_obj->isKinematic()) {
                    if (jsonNodeExists(rb, "linear_damping", nlohmann::json::value_t::number_float)) rb_obj->setLinearDamping(rb["linear_damping"]);
                    if (jsonNodeExists(rb, "angular_damping", nlohmann::json::value_t::number_float)) rb_obj->setAngularDamping(rb["angular_damping"]);
                    if (jsonArrayExists(rb, "linear_velocity", 3, nlohmann::json::value_t::number_float))
                        rb_obj->setLinearVelocity(pe::Vector3(rb["linear_velocity"][0], rb["linear_velocity"][1], rb["linear_velocity"][2]));
                    if (jsonArrayExists(rb, "angular_velocity", 3, nlohmann::json::value_t::number_float))
                        rb_obj->setAngularVelocity(pe::Vector3(rb["angular_velocity"][0], rb["angular_velocity"][1], rb["angular_velocity"][2]));
                }
                if (jsonNodeExists(rb, "lifetime", nlohmann::json::value_t::number_float)) {
                    auto lifetime = rb["lifetime"].get<pe::Real>();
                    lifetime = lifetime < 0 ? PE_REAL_MAX : lifetime;
                    rb_obj->setLifeTime(lifetime);
                }
                auto transform = pe::Transform::identity();
                if (jsonArrayExists(rb, "position", 0, nlohmann::json::value_t::number_float))
                    transform.setOrigin(pe::Vector3(rb["position"][0], rb["position"][1], rb["position"][2]));
                if (jsonArrayExists(rb, "rotation", 0, nlohmann::json::value_t::number_float)) {
                    if (rb["rotation"].size() >= 9) {
                        auto m = pe::Matrix3(rb["rotation"][0], rb["rotation"][1], rb["rotation"][2],
                                             rb["rotation"][3], rb["rotation"][4], rb["rotation"][5],
                                             rb["rotation"][6], rb["rotation"][7], rb["rotation"][8]);
                        transform.setBasis(m);
                    } else if (rb["rotation"].size() >= 4) {
                        auto q = pe::Quaternion(rb["rotation"][0], rb["rotation"][1], rb["rotation"][2], rb["rotation"][3]);
                        transform.setBasis(q.toRotationMatrix());
                    } else {
                        throw std::runtime_error("Invalid rotation data.");
                    }
                }
                rb_obj->setTransform(transform);
                if (jsonNodeExists(rb, "mass", nlohmann::json::value_t::number_float)) rb_obj->setMass(rb["mass"]);
                if (jsonNodeExists(rb, "friction", nlohmann::json::value_t::number_float)) rb_obj->setFrictionCoeff(rb["friction"]);
                if (jsonNodeExists(rb, "restitution", nlohmann::json::value_t::number_float)) rb_obj->setRestitutionCoeff(rb["restitution"]);
                if (jsonNodeExists(rb, "type", nlohmann::json::value_t::string)) {
                    auto type = rb["type"].get<std::string>();
                    pe_phys_shape::Shape* shape;
                    if (type == "box") {
                        pe::Vector3 size = {1, 1, 1};
                        if (jsonArrayExists(rb, "scale", 3, nlohmann::json::value_t::number_float)) size = pe::Vector3(rb["scale"][0], rb["scale"][1], rb["scale"][2]);
                        shape = new pe_phys_shape::BoxShape(size);
                    } else if (type == "sphere") {
                        pe::Real radius = 1;
                        if (jsonArrayExists(rb, "scale", 1, nlohmann::json::value_t::number_float)) radius = rb["scale"][0].get<pe::Real>();
                        shape = new pe_phys_shape::SphereShape(radius);
                    } else if (type == "cylinder") {
                        pe::Real radius = 0.5, height = 1;
                        if (jsonArrayExists(rb, "scale", 2, nlohmann::json::value_t::number_float)) {
                            radius = rb["scale"][0].get<pe::Real>();
                            height = rb["scale"][1].get<pe::Real>();
                        }
                        shape = new pe_phys_shape::CylinderShape(radius, height);
                    } else if (type == "convex" || type == "concave") {
                        if (jsonNodeExists(rb, "mesh", nlohmann::json::value_t::string)) {
                            auto obj_path = rb["mesh"].get<std::string>();
                            pe::Vector3 size = {1, 1, 1};
                            if (jsonArrayExists(rb, "scale", 3, nlohmann::json::value_t::number_float)) size = pe::Vector3(rb["scale"][0], rb["scale"][1], rb["scale"][2]);
                            if (!utils::FileSystem::fileExists(obj_path)) {
                                PE_LOG_CUSTOM_ERROR << "Mesh file not found: " << obj_path << PE_CUSTOM_ENDL;
                                delete rb_obj;
                                continue;
                            }
                            pe::Mesh mesh;
                            pe::Mesh::loadFromObj(obj_path, mesh, size);
                            if (type == "convex") {
                                shape = new pe_phys_shape::ConvexMeshShape();
                                ((pe_phys_shape::ConvexMeshShape*)shape)->setMesh(mesh);
                                ((pe_phys_shape::ConvexMeshShape*)shape)->setMeshPath(obj_path);
                                ((pe_phys_shape::ConvexMeshShape*)shape)->setScale(size);
                            } else {
                                shape = new pe_phys_shape::ConcaveMeshShape();
                                ((pe_phys_shape::ConcaveMeshShape*)shape)->setMesh(mesh);
                                ((pe_phys_shape::ConcaveMeshShape*)shape)->setMeshPath(obj_path);
                                ((pe_phys_shape::ConcaveMeshShape*)shape)->setScale(size);
                            }
                        } else {
                            PE_LOG_CUSTOM_ERROR << "Mesh path not specified." << PE_CUSTOM_ENDL;
                            delete rb_obj;
                            continue;
                        }
                    } else if (type == "compound") {
                        if (jsonArrayExists(rb, "shapes", 0, nlohmann::json::value_t::object)) {
                            shape = new pe_phys_shape::CompoundShape();
                            auto& shapes = rb["shapes"];
                            if (shapes.type() != nlohmann::json::value_t::array) {
                                PE_LOG_CUSTOM_ERROR << "Compound sub-shapes should be an array." << PE_CUSTOM_ENDL;
                                delete rb_obj;
                                continue;
                            }
                            for (auto& s : shapes) {
                                pe::Real mass_ratio = 1.0;
                                pe::Transform local_transform = pe::Transform::identity();
                                if (jsonNodeExists(s, "mass_ratio", nlohmann::json::value_t::number_float)) mass_ratio = s["mass_ratio"];
                                if (jsonArrayExists(s, "position", 3, nlohmann::json::value_t::number_float))
                                    local_transform.setOrigin(pe::Vector3(s["position"][0], s["position"][1], s["position"][2]));
                                if (jsonArrayExists(s, "rotation", 0, nlohmann::json::value_t::number_float)) {
                                    if (s["rotation"].size() >= 9) {
                                        auto m = pe::Matrix3(s["rotation"][0], s["rotation"][1], s["rotation"][2],
                                                             s["rotation"][3], s["rotation"][4], s["rotation"][5],
                                                             s["rotation"][6], s["rotation"][7], s["rotation"][8]);
                                        local_transform.setBasis(m);
                                    } else if (s["rotation"].size() >= 4) {
                                        auto q = pe::Quaternion(s["rotation"][0], s["rotation"][1], s["rotation"][2], s["rotation"][3]);
                                        local_transform.setBasis(q.toRotationMatrix());
                                    } else  {
                                        throw std::runtime_error("Invalid rotation data.");
                                    }
                                }
                                pe_phys_shape::Shape* sub_shape;
                                if (jsonNodeExists(s, "type", nlohmann::json::value_t::string)) {
                                    auto sub_type = s["type"].get<std::string>();
                                    if (sub_type == "box") {
                                        pe::Vector3 size = {1, 1, 1};
                                        if (jsonArrayExists(s, "scale", 3, nlohmann::json::value_t::number_float))
                                            size = pe::Vector3(s["scale"][0], s["scale"][1], s["scale"][2]);
                                        sub_shape = new pe_phys_shape::BoxShape(size);
                                    } else if (sub_type == "sphere") {
                                        pe::Real radius = 1;
                                        if (jsonArrayExists(s, "scale", 1, nlohmann::json::value_t::number_float))
                                            radius = s["scale"][0].get<pe::Real>();
                                        sub_shape = new pe_phys_shape::SphereShape(radius);
                                    } else if (sub_type == "cylinder") {
                                        pe::Real radius = 0.5, height = 1;
                                        if (jsonArrayExists(s, "scale", 2, nlohmann::json::value_t::number_float)) {
                                            radius = s["scale"][0].get<pe::Real>();
                                            height = s["scale"][1].get<pe::Real>();
                                        }
                                        sub_shape = new pe_phys_shape::CylinderShape(radius, height);
                                    } else if (sub_type == "convex") {
                                        if (jsonNodeExists(s, "mesh", nlohmann::json::value_t::string)) {
                                            auto obj_path = s["mesh"].get<std::string>();
                                            auto size = pe::Vector3(s["scale"][0], s["scale"][1], s["scale"][2]);
                                            pe::Mesh mesh;
                                            pe::Mesh::loadFromObj(obj_path, mesh, size);
                                            sub_shape = new pe_phys_shape::ConvexMeshShape();
                                            ((pe_phys_shape::ConvexMeshShape*)sub_shape)->setMesh(mesh);
                                            ((pe_phys_shape::ConvexMeshShape*)sub_shape)->setMeshPath(obj_path);
                                        } else {
                                            PE_LOG_CUSTOM_ERROR << "Convex sub-shape mesh path not specified." << PE_CUSTOM_ENDL;
                                            delete rb_obj;
                                            continue;
                                        }
                                    } else if (sub_type == "compound" || sub_type == "concave") {
                                        PE_LOG_CUSTOM_ERROR << "Compound sub-shapes cannot be compound or concave." << PE_CUSTOM_ENDL;
                                        delete rb_obj;
                                        continue;
                                    } else {
                                        PE_LOG_CUSTOM_ERROR << "Invalid compound sub-shape type: " << sub_type << PE_CUSTOM_ENDL;
                                        delete rb_obj;
                                        continue;
                                    }
                                    ((pe_phys_shape::CompoundShape*)shape)->addShape(local_transform, mass_ratio, sub_shape);
                                } else {
                                    PE_LOG_CUSTOM_ERROR << "Compound sub-shape type not specified." << PE_CUSTOM_ENDL;
                                    delete rb_obj;
                                    continue;
                                }
                            }
                        } else {
                            PE_LOG_CUSTOM_ERROR << "Compound sub-shapes not specified." << PE_CUSTOM_ENDL;
                            delete rb_obj;
                            continue;
                        }
                    } else {
                        PE_LOG_CUSTOM_ERROR << "Invalid rigidbody type: " << type << PE_CUSTOM_ENDL;
                        delete rb_obj;
                        continue;
                    }
                    rb_obj->setCollisionShape(shape);
                } else {
                    PE_LOG_CUSTOM_ERROR << "RigidBody type not specified." << PE_CUSTOM_ENDL;
                    delete rb_obj;
                    continue;
                }
                _world.addRigidBody(rb_obj);
                k++;
            }
        }
        PE_LOG_CUSTOM_INFO << "Loaded " << k << " rigidbodies." << PE_CUSTOM_ENDL;

        //////////////////////////////////////////////////////////////////////////
        // read damage sources
        //////////////////////////////////////////////////////////////////////////
        k = 0;
        if (jsonArrayExists(data, "DamageSources", 0, nlohmann::json::value_t::object)) {
            auto& dss = data["DamageSources"];
            if (dss.type() != nlohmann::json::value_t::array) {
                PE_LOG_CUSTOM_ERROR << "DamageSources should be an array." << PE_CUSTOM_ENDL;
                return false;
            }
            for (auto& ds : dss) {
                pe_phys_fracture::FractureSource source;
                if (jsonNodeExists(ds, "type", nlohmann::json::value_t::string)) {
                    auto type = ds["type"].get<std::string>();
                    if (type == "sphere") {
                        source.type = pe_phys_fracture::FractureType::Sphere;
                    } else if (type == "cylinder") {
                        source.type = pe_phys_fracture::FractureType::Cylinder;
                    } else {
                        PE_LOG_CUSTOM_ERROR << "Invalid damage source type: " << type << PE_CUSTOM_ENDL;
                        continue;
                    }
                } else {
                    PE_LOG_CUSTOM_ERROR << "Damage source type not specified." << PE_CUSTOM_ENDL;
                    continue;
                }
                if (jsonArrayExists(ds, "position", 3, nlohmann::json::value_t::number_float))
                    source.position = pe::Vector3(ds["position"][0], ds["position"][1], ds["position"][2]);
                if (jsonArrayExists(ds, "intensity", 3, nlohmann::json::value_t::number_float))
                    source.intensity = pe::Vector3(ds["intensity"][0], ds["intensity"][1], ds["intensity"][2]);
                _world.addFractureSource(source);
                k++;
            }
        }

        //////////////////////////////////////////////////////////////////////////
        // read constraints
        //////////////////////////////////////////////////////////////////////////
        // not implemented

        PE_LOG_CUSTOM_INFO << "Scene loaded." << PE_CUSTOM_ENDL;
        return true;
    }

#ifdef PE_USE_DOUBLE
#define TINYGLTF_COMPONENT_TYPE_REAL TINYGLTF_COMPONENT_TYPE_FLOAT
#else
#define TINYGLTF_COMPONENT_TYPE_REAL TINYGLTF_COMPONENT_TYPE_FLOAT
#endif

    void Simulator::saveGltf(const std::string &path_to_write_gltf) {
        class GltfWriter {
            std::string _path;
            tinygltf::Model _model;
            tinygltf::Node _root;
            tinygltf::Buffer _buffer_position;
            tinygltf::Buffer _buffer_normal;
            tinygltf::Buffer _buffer_index;
            tinygltf::Buffer _buffer_animation_time_step;
            tinygltf::Buffer _buffer_animation_translation;
            tinygltf::Buffer _buffer_animation_rotation;

            uint32_t _mesh_counter = 0;
            uint32_t _accessor_counter = 0;
            size_t _buffer_position_offset = 0;
            size_t _buffer_normal_offset = 0;
            size_t _buffer_index_offset = 0;
            size_t _buffer_animation_time_step_offset = 0;
            size_t _buffer_animation_translation_offset = 0;
            size_t _buffer_animation_rotation_offset = 0;

            pe::Map<uint32_t, uint32_t> _mesh2nodeId;

            pe::Array<pe::Array<pe::Vector3>> _animation_translation;
            pe::Array<pe::Array<pe::Quaternion>> _animation_rotation;
            pe::Array<pe::Array<pe::Real>> _animation_time_step;

        public:
            explicit GltfWriter(const std::string &path) {
                _path = path;

                _model = tinygltf::Model();
                _model.asset.version = "2.0";
                _model.asset.generator = "RigidBody Dynamics - DALAB";

                _root = tinygltf::Node();
                _root.name = "root";
            }
            ~GltfWriter() {
                _model.nodes.push_back(_root);

                tinygltf::Scene scene;
                scene.name = "rigidbody scene";
                scene.nodes.push_back(_mesh_counter);
                _model.scenes.push_back(scene);
                _model.defaultScene = 0;

                tinygltf::Animation animation;
                animation.name = "animation";
                int anim_count = (int)_animation_translation.size();
                for (int i = 0; i < anim_count; i++) {
                    tinygltf::AnimationChannel channel_translation;
                    channel_translation.target_path = "translation";
                    channel_translation.target_node = i;
                    channel_translation.sampler = i * 2;
                    tinygltf::AnimationChannel channel_rotation;
                    channel_rotation.target_path = "rotation";
                    channel_rotation.target_node = i;
                    channel_rotation.sampler = i * 2 + 1;
                    tinygltf::AnimationSampler sampler_translation;
                    sampler_translation.input = _accessor_counter + 2;
                    sampler_translation.output = _accessor_counter;
                    tinygltf::AnimationSampler sampler_rotation;
                    sampler_rotation.input = _accessor_counter + 2;
                    sampler_rotation.output = _accessor_counter + 1;
                    animation.channels.push_back(channel_translation);
                    animation.channels.push_back(channel_rotation);
                    animation.samplers.push_back(sampler_translation);
                    animation.samplers.push_back(sampler_rotation);

                    tinygltf::Accessor translation_accessor;
                    translation_accessor.bufferView = _accessor_counter;
                    translation_accessor.componentType = TINYGLTF_COMPONENT_TYPE_REAL;
                    translation_accessor.count = _animation_translation[i].size();
                    translation_accessor.type = TINYGLTF_TYPE_VEC3;
                    _model.accessors.push_back(translation_accessor);
                    tinygltf::BufferView translation_buffer_view;
                    translation_buffer_view.buffer = 3;
                    translation_buffer_view.byteOffset = _buffer_animation_translation_offset;
                    _buffer_animation_translation_offset += translation_accessor.count * sizeof(float) * 3;
                    translation_buffer_view.byteLength = translation_accessor.count * sizeof(float) * 3;
                    _model.bufferViews.push_back(translation_buffer_view);
                    pe::Array<float> data;
                    data.reserve(translation_accessor.count * 3);
                    for (auto& v : _animation_translation[i]) {
                        data.push_back(v.x);
                        data.push_back(v.y);
                        data.push_back(v.z);
                    }
                    _buffer_animation_translation.data.insert(_buffer_animation_translation.data.end(), (const uint8_t*)data.data(), (const uint8_t*)data.data() + data.size() * sizeof(float));

                    tinygltf::Accessor rotation_accessor;
                    rotation_accessor.bufferView = _accessor_counter + 1;
                    rotation_accessor.componentType = TINYGLTF_COMPONENT_TYPE_REAL;
                    rotation_accessor.count = _animation_rotation[i].size();
                    rotation_accessor.type = TINYGLTF_TYPE_VEC4;
                    _model.accessors.push_back(rotation_accessor);
                    tinygltf::BufferView rotation_buffer_view;
                    rotation_buffer_view.buffer = 4;
                    rotation_buffer_view.byteOffset = _buffer_animation_rotation_offset;
                    _buffer_animation_rotation_offset += rotation_accessor.count * sizeof(float) * 4;
                    rotation_buffer_view.byteLength = rotation_accessor.count * sizeof(float) * 4;
                    _model.bufferViews.push_back(rotation_buffer_view);
                    data.clear();
                    data.reserve(rotation_accessor.count * 4);
                    for (auto& q : _animation_rotation[i]) {
                        data.push_back(q.x);
                        data.push_back(q.y);
                        data.push_back(q.z);
                        data.push_back(q.w);
                    }
                    _buffer_animation_rotation.data.insert(_buffer_animation_rotation.data.end(), (const uint8_t*)data.data(), (const uint8_t*)data.data() + data.size() * sizeof(float));

                    tinygltf::Accessor time_step_accessor;
                    time_step_accessor.bufferView = _accessor_counter + 2;
                    time_step_accessor.componentType = TINYGLTF_COMPONENT_TYPE_REAL;
                    time_step_accessor.count = _animation_time_step[i].size();
                    time_step_accessor.type = TINYGLTF_TYPE_SCALAR;
                    _model.accessors.push_back(time_step_accessor);
                    tinygltf::BufferView time_step_buffer_view;
                    time_step_buffer_view.buffer = 5;
                    time_step_buffer_view.byteOffset = _buffer_animation_time_step_offset;
                    _buffer_animation_time_step_offset += time_step_accessor.count * sizeof(float);
                    time_step_buffer_view.byteLength = time_step_accessor.count * sizeof(float);
                    _model.bufferViews.push_back(time_step_buffer_view);
                    data.clear();
                    data.reserve(time_step_accessor.count);
                    for (auto& t : _animation_time_step[i]) {
                        data.push_back(t);
                    }
                    _buffer_animation_time_step.data.insert(_buffer_animation_time_step.data.end(), (const uint8_t*)data.data(), (const uint8_t*)data.data() + data.size() * sizeof(float));

                    _accessor_counter += 3;
                }
                _model.animations.push_back(animation);

                _model.buffers.push_back(_buffer_position);
                _model.buffers.push_back(_buffer_normal);
                _model.buffers.push_back(_buffer_index);
                _model.buffers.push_back(_buffer_animation_translation);
                _model.buffers.push_back(_buffer_animation_rotation);
                _model.buffers.push_back(_buffer_animation_time_step);

                tinygltf::TinyGLTF gltf;
                gltf.WriteGltfSceneToFile(&_model, _path, true, true, true, false);
            }

            bool isTrackingMesh(const uint32_t id) {
                return _mesh2nodeId.find(id) != _mesh2nodeId.end();
            }

            void addMesh(const pe::Mesh& mesh, const uint32_t id) {
                tinygltf::Node node;
                node.name = "node-" + std::to_string(_mesh_counter);
                node.mesh = _mesh_counter;
                _root.children.push_back(_mesh_counter);
                _mesh2nodeId[id] = _mesh_counter;

                tinygltf::Mesh gltf_mesh;
                gltf_mesh.name = "mesh-" + std::to_string(_mesh_counter);
                tinygltf::Primitive primitive;
                primitive.mode = TINYGLTF_MODE_TRIANGLES;
                primitive.attributes["POSITION"] = _accessor_counter;
                primitive.attributes["NORMAL"] = _accessor_counter + 1;
                primitive.indices = _accessor_counter + 2;
                gltf_mesh.primitives.push_back(primitive);
                _model.meshes.push_back(gltf_mesh);

                tinygltf::Accessor position_accessor;
                position_accessor.bufferView = _accessor_counter;
                position_accessor.componentType = TINYGLTF_COMPONENT_TYPE_REAL;
                position_accessor.count = mesh.vertices.size();
                position_accessor.type = TINYGLTF_TYPE_VEC3;
                _model.accessors.push_back(position_accessor);
                tinygltf::BufferView position_buffer_view;
                position_buffer_view.buffer = 0;
                position_buffer_view.byteOffset = _buffer_position_offset;
                _buffer_position_offset += position_accessor.count * sizeof(float) * 3;
                position_buffer_view.byteLength = position_accessor.count * sizeof(float) * 3;
                _model.bufferViews.push_back(position_buffer_view);
                pe::Array<float> data;
                data.reserve(position_accessor.count * 3);
                for (auto& v : mesh.vertices) {
                    data.push_back(v.position.x);
                    data.push_back(v.position.y);
                    data.push_back(v.position.z);
                }
                _buffer_position.data.insert(_buffer_position.data.end(), (const uint8_t*)data.data(), (const uint8_t*)data.data() + data.size() * sizeof(float));

                tinygltf::Accessor normal_accessor;
                normal_accessor.bufferView = _accessor_counter + 1;
                normal_accessor.componentType = TINYGLTF_COMPONENT_TYPE_REAL;
                normal_accessor.count = mesh.vertices.size();
                normal_accessor.type = TINYGLTF_TYPE_VEC3;
                _model.accessors.push_back(normal_accessor);
                tinygltf::BufferView normal_buffer_view;
                normal_buffer_view.buffer = 1;
                normal_buffer_view.byteOffset = _buffer_normal_offset;
                _buffer_normal_offset += normal_accessor.count * sizeof(float) * 3;
                normal_buffer_view.byteLength = normal_accessor.count * sizeof(float) * 3;
                _model.bufferViews.push_back(normal_buffer_view);
                data.clear();
                data.reserve(normal_accessor.count * 3);
                for (auto& v : mesh.vertices) {
                    data.push_back(v.normal.x);
                    data.push_back(v.normal.y);
                    data.push_back(v.normal.z);
                }
                _buffer_normal.data.insert(_buffer_normal.data.end(), (const uint8_t*)data.data(), (const uint8_t*)data.data() + data.size() * sizeof(float));

                tinygltf::Accessor index_accessor;
                index_accessor.bufferView = _accessor_counter + 2;
                index_accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
                index_accessor.count = 0;
                for (auto& f : mesh.faces) {
                    index_accessor.count += (f.indices.size() - 2) * 3;
                }
                index_accessor.type = TINYGLTF_TYPE_SCALAR;
                _model.accessors.push_back(index_accessor);
                tinygltf::BufferView index_buffer_view;
                index_buffer_view.buffer = 2;
                index_buffer_view.byteOffset = _buffer_index_offset;
                _buffer_index_offset += index_accessor.count * sizeof(uint32_t);
                index_buffer_view.byteLength = index_accessor.count * sizeof(uint32_t);
                _model.bufferViews.push_back(index_buffer_view);
                pe::Array<uint32_t> indices;
                indices.reserve(index_accessor.count);
                for (auto& f : mesh.faces) {
                    for (size_t i = 1; i < f.indices.size() - 1; i++) {
                        indices.push_back(f.indices[0]);
                        indices.push_back(f.indices[i]);
                        indices.push_back(f.indices[i + 1]);
                    }
                }
                _buffer_index.data.insert(_buffer_index.data.end(), (const uint8_t*)indices.data(), (const uint8_t*)indices.data() + indices.size() * sizeof(uint32_t));

                _model.nodes.push_back(node);
                _animation_translation.push_back(pe::Array<pe::Vector3>());
                _animation_rotation.push_back(pe::Array<pe::Quaternion>());
                _animation_time_step.push_back(pe::Array<pe::Real>());
                _mesh_counter++;
                _accessor_counter += 3;
            }

            void addAnimation(const uint32_t id, const pe::Transform& transform, pe::Real time_step) {
                auto& anim_trans = _animation_translation[_mesh2nodeId[id]];
                auto& anim_rot = _animation_rotation[_mesh2nodeId[id]];
                auto& anim_time = _animation_time_step[_mesh2nodeId[id]];
                anim_trans.push_back(transform.getOrigin());
                anim_rot.push_back(pe::Quaternion::fromRotationMatrix(transform.getBasis()));
                anim_time.push_back(time_step);
            }

        } static writer(path_to_write_gltf);

        static int frame = 0;
        for (auto rb : _world.getRigidBodies()) {
            auto shape = rb->getCollisionShape();
            switch (shape->getType()) {
                case pe_phys_shape::ShapeType::Box: {
                    auto& mesh = ((pe_phys_shape::BoxShape*)shape)->getMesh();
                    if (!writer.isTrackingMesh(shape->getGlobalId())) {
                        writer.addMesh(mesh, shape->getGlobalId());
                    }
                    writer.addAnimation(shape->getGlobalId(), rb->getTransform(), _world.getDt() * frame);
                    break;
                }
                case pe_phys_shape::Cylinder: {
                    auto& mesh = ((pe_phys_shape::CylinderShape*)shape)->getMesh();
                    if (!writer.isTrackingMesh(shape->getGlobalId())) {
                        writer.addMesh(mesh, shape->getGlobalId());
                    }
                    writer.addAnimation(shape->getGlobalId(), rb->getTransform(), _world.getDt() * frame);
                    break;
                }
                case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh: {
                    auto& mesh = ((pe_phys_shape::ConvexMeshShape*)shape)->getMesh();
                    if (!writer.isTrackingMesh(shape->getGlobalId())) {
                        writer.addMesh(mesh, shape->getGlobalId());
                    }
                    writer.addAnimation(shape->getGlobalId(), rb->getTransform(), _world.getDt() * frame);
                    break;
                }

                case pe_phys_shape::ShapeType::Sphere: {
                    static pe::Mesh sphere_mesh = PE_SPHERE_DEFAULT_MESH;
                    if (!writer.isTrackingMesh(shape->getGlobalId())) {
                        writer.addMesh(sphere_mesh, shape->getGlobalId());
                    }
                    writer.addAnimation(shape->getGlobalId(), rb->getTransform(), _world.getDt() * frame);
                    break;
                }
                default:
                    break;
            }
        }
        frame++;
    }

    static void printWelcomeMessage() {
        PE_LOG_CUSTOM_INFO << "Press `x` to start simulation" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `r` to simulate for a period while pressing" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `t` to simulate a single step" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `c` to show edges" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `n` to download the simulation data to json files (default in ./data/)" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `esc` to exit" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Camera control: UE mode" << PE_CUSTOM_ENDL;
    }

    void Simulator::start() {
        printWelcomeMessage();

        if (target_framerate <= 0) {
            PE_LOG_CUSTOM_ERROR << "Invalid target frame rate: " << target_framerate << PE_CUSTOM_ENDL;
            return;
        }

        pe::Real dt = pe::Real(1.0) / (pe::Real)target_framerate;
        _world.setDt(dt);
        init();
        if (use_gui) {
            if (!renderInit()) {
                return;
            }
        }

        int frame = 0;
        int target_dt = (int)(dt * 1000);
        auto start = COMMON_GetTickCount();

	    pe::Real total_step_time = 0;
        while (true) {
            auto t = COMMON_GetTickCount();

		    auto step_start = COMMON_GetMicroseconds();
            _world.step();
		    auto step_end = COMMON_GetMicroseconds();
		    total_step_time += (pe::Real)(step_end - step_start);

            if (use_gui) {
                if (!_world.getRigidBodiesToRemove().empty()) {
                    removeModels(_world.getRigidBodiesToRemove());
                    _world.clearRigidBodiesToRemove();
                }
                if (!_world.getRigidBodiesToAdd().empty()) {
                    addModels(_world.getRigidBodiesToAdd());
                    _world.clearRigidBodiesToAdd();
                }
            }

            step();

            if (use_gui) {
                if (!renderStep()) {
                    break;
                }
            }

            static int saved_file_count = 0;
            static int program_tick = COMMON_GetTickCount();
            if ((use_gui && Viewer::getKeyState('n') == 0) || _saving) {
                if (!utils::FileSystem::isDirectory(save_path)) {
                    utils::FileSystem::makeDir(save_path);
                }
                auto dir = save_path + "/program-" + std::to_string(program_tick) + "/";
                auto filename = std::to_string(saved_file_count++) + "-" + std::to_string(COMMON_GetTickCount()) + ".json";
                saveScene(dir + filename);
                auto filename_gltf = std::to_string(COMMON_GetTickCount()) + ".gltf";
                saveGltf(dir + filename_gltf);
            }

            auto actual_dt = (int)(COMMON_GetTickCount() - t);
            if (target_dt > actual_dt) {
                COMMON_Sleep(target_dt - actual_dt);
            }
            if (frame++ >= max_frame) {
                break;
            }

    #   if false
            static pe::Array<int> ids;
            for (auto id : ids) {
                Viewer::remove(id);
            }
            ids.clear();
            for (auto cr : _world.getContactResults()) {
                for (int i = 0; i < cr->getPointSize(); i++) {
                    auto p = cr->getContactPoint(i).getWorldPos();
                    auto id = Viewer::addSphere(0.1);
                    Viewer::updateTransform(id, pe_phys_shape::ShapeType::Sphere, pe::Transform(pe::Matrix3::identity(), p));
                    Viewer::updateColor(id, pe_phys_shape::ShapeType::Sphere, pe::Vector3(0.8, 0.3, 0.3));
                    ids.push_back(id);
                }
            }
    #   endif
        }

        auto end = COMMON_GetTickCount();
        pe::Real total_time = (pe::Real)(end - start) * pe::Real(0.001);
        pe::Real step_time = total_step_time * pe::Real(0.000001);
	    std::cout << "step time: " << step_time << "s" << std::endl;
        std::cout << "frame count: " << frame << ", simulation fps: " << (pe::Real)frame / step_time << std::endl;
        std::cout << "total time: " << total_time << "s" << std::endl;
        std::cout << "update status time: " << _world.update_status_time << "s " << _world.update_status_time / total_time << std::endl;
        std::cout << "broad phase time: " << _world.broad_phase_time << "s " << _world.broad_phase_time / total_time << std::endl;
        std::cout << "narrow phase time: " << _world.narrow_phase_time << "s " << _world.narrow_phase_time / total_time << std::endl;
        std::cout << "constraint solver time: " << _world.constraint_solver_time << "s " << _world.constraint_solver_time / total_time << std::endl;
        pe::Real other_time = total_time - _world.update_status_time - _world.broad_phase_time - _world.narrow_phase_time - _world.constraint_solver_time;
        std::cout << "other time: " << other_time << "s" << " " << other_time / total_time << std::endl;
    }

    static void toggleLine() {
        static bool show_line = false;
        if (Viewer::getKeyState('c') == 2) {
            show_line = !show_line;
            Viewer::showLine(show_line, 1);
        }
    }

    bool Simulator::renderInit() {
        Viewer::open("PhysicsDemo", 800, 600, {0, 10, 20}, 0, (float)(PE_PI / 12.0));

        // initialize models
        addModels(_world.getRigidBodiesToAdd());
        _world.clearRigidBodiesToAdd();

        // wait for the window to open
        while (!Viewer::isOpen()) {
            COMMON_Sleep(10);
        }

        return true;
    }

    bool Simulator::renderStep() {
        if (!Viewer::isOpen() || Viewer::getKeyState(27) == 0) {
            Viewer::close();
            return false;
        }
        static bool blocking = true;
        if (blocking) {
            while (Viewer::getKeyState('r') != 0 && Viewer::getKeyState('t') != 0) {
                COMMON_Sleep(1);
                toggleLine();
                if (Viewer::getKeyState('x') == 2) {
                    blocking = false;
                    break;
                }
                if (!Viewer::isOpen() || Viewer::getKeyState(27) == 0) {
                    Viewer::close();
                    return false;
                }
            }
            if (Viewer::getKeyState('t') == 0) {
                COMMON_Sleep(300);
            }
        } else {
            toggleLine();
        }

        for (auto& rb : _id_map) {
            if (rb.second.empty()) {
                continue;
            }
            auto type = rb.first->getCollisionShape()->getType();
            if (type != pe_phys_shape::ShapeType::Compound) {
                updateColor(rb.second[0], type, rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
                if (!rb.first->isSleep()) {
                    Viewer::updateTransform(rb.second[0], type, rb.first->getTransform());
                }
            } else {
                int i = 0;
                for (auto& s : ((pe_phys_shape::CompoundShape*)rb.first->getCollisionShape())->getShapes()) {
                    updateColor(rb.second[i], s.shape->getType(), rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
                    if (!rb.first->isSleep()) {
                        Viewer::updateTransform(rb.second[i], s.shape->getType(), rb.first->getTransform() * s.local_transform);
                    }
                    i++;
                }
            }
        }
        return true;
    }

    void Simulator::addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        // add models and initialize transform
        for (auto rb : rbs) {
            pe::Array<int> ids;
            auto type = rb->getCollisionShape()->getType();
            switch (type) {
                case pe_phys_shape::ShapeType::Box:
                    ids.push_back(Viewer::addCube(((pe_phys_shape::BoxShape*)rb->getCollisionShape())->getSize()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], type, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::Sphere:
                    ids.push_back(Viewer::addSphere(((pe_phys_shape::SphereShape*)rb->getCollisionShape())->getRadius()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], pe_phys_shape::ShapeType::Sphere, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::Cylinder:
                    ids.push_back(Viewer::addCylinder(((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getRadius(),
                                                               ((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getHeight()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], pe_phys_shape::ShapeType::Cylinder, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
                    ids.push_back(Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)rb->getCollisionShape())->getMesh()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], pe_phys_shape::ShapeType::ConvexMesh, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::Compound:
                    int i = 0;
                    for (auto& s : ((pe_phys_shape::CompoundShape*)rb->getCollisionShape())->getShapes()) {
                        auto sub_type = s.shape->getType();
                        switch (sub_type) {
                            case pe_phys_shape::ShapeType::Box:
                                ids.push_back(Viewer::addCube(((pe_phys_shape::BoxShape*)s.shape)->getSize()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                                break;
                            case pe_phys_shape::ShapeType::Sphere:
                                ids.push_back(Viewer::addSphere(((pe_phys_shape::SphereShape*)s.shape)->getRadius()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                                break;
                            case pe_phys_shape::ShapeType::Cylinder:
                                ids.push_back(Viewer::addCylinder(((pe_phys_shape::CylinderShape*)s.shape)->getRadius(),
                                                                           ((pe_phys_shape::CylinderShape*)s.shape)->getHeight()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                                break;
                            case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
                                ids.push_back(Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)s.shape)->getMesh()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                            default:
                                break;
                        }
                    }
                    break;
            }
            _id_map[rb] = ids;
        }
    }

    void Simulator::updateColor(int id, pe_phys_shape::ShapeType type, const std::string &tag, bool kinematic) {
        auto color_p = tag.find("color:");
        if (color_p != std::string::npos) {
            std::stringstream ss(tag.substr(color_p + 6));
            pe::Real r, g, b;
            char delim;
            ss >> r >> delim >> g >> delim >> b;
            if (ss.good() || ss.eof()) {
                Viewer::updateColor(id, type, pe::Vector3(r, g, b));
                return;
            } else {
                PE_LOG_CUSTOM_ERROR << "invalid color tag" << PE_CUSTOM_ENDL;
            }
        }

        if (kinematic) {
            Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.8), pe::Real(0.8)));
            return;
        }

        switch (type) {
            case pe_phys_shape::ShapeType::Box:
                Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.3), pe::Real(0.8)));
                break;
            case pe_phys_shape::ShapeType::Sphere:
                Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.8), pe::Real(0.3), pe::Real(0.3)));
                break;
            case pe_phys_shape::ShapeType::Cylinder:
                Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.8), pe::Real(0.3)));
                break;
            case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
                Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.8), pe::Real(0.8), pe::Real(0.3)));
            default:
                break;
        }
    }

    void Simulator::removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        for (auto rb : rbs) {
            if (_id_map.find(rb) != _id_map.end()) {
                for (auto id : _id_map[rb]) {
                    Viewer::remove(id);
                }
                _id_map[rb] = {};
            }
        }
    }
}
