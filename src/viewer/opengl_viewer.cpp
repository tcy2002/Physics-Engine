#include "opengl_viewer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <mutex>
#include <atomic>
#include "camera.h"
#include "renderer.h"
#include "shader_program.h"
#include "shader_vert.h"
#include "shader_frag.h"

using namespace pe_viewer;

//// camera
static std::atomic<Camera*> camera = nullptr;

//// shader
static ShaderProgram* shader = nullptr;

//// object
static pe::Array<pe::Pair<int, Renderer*>> objs;
static int max_id = -1;
static std::mutex mtx;

static void drawMesh(MeshRenderer* mesh) {
    // set mesh properties
    auto& transform = mesh->getTransform();
    shader->setMat3("gWorldBasis", transform.getBasis());
    shader->setVec3("gWorldOrigin", transform.getOrigin());
    shader->setVec3("gColor", mesh->getColor());
    shader->setFloat("gAmbientIntensity", 0.8f);
    shader->setFloat("gDiffuseIntensity", 0.4f);

    // render mesh
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    mesh->render();
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
}

static void drawLine(LineRenderer* line) {
    // set line properties
    shader->setMat3("gWorldBasis", pe_common::Matrix3x3::identity());
    shader->setVec3("gWorldOrigin", pe_common::Vector3::zeros());
    shader->setVec3("gColor", line->getColor());
    shader->setFloat("gAmbientIntensity", 1.0f);
    shader->setFloat("gDiffuseIntensity", 0);

    // render line
    glEnableVertexAttribArray(1);
    line->render();
    glDisableVertexAttribArray(1);
}

static void drawObjects() {
    std::unique_lock<std::mutex> lock(mtx);
    for (int i = (int)objs.size() - 1; i >= 0; i--) {
        // update object
        if (objs[i].first == -1) {
            objs[i].second->deinit();
            delete objs[i].second;
            objs.erase(objs.begin() + i);
            continue;
        } else if (!objs[i].second->isInited()) {
            objs[i].second->init(1, 2);
        }
        if (objs[i].second->type() == RenderType::R_MESH) {
            drawMesh(dynamic_cast<MeshRenderer*>(objs[i].second));
        } else if (objs[i].second->type() == RenderType::R_LINE) {
            drawLine(dynamic_cast<LineRenderer*>(objs[i].second));
        }
    }
}

static void display() {
    if (camera == nullptr || shader == nullptr) return;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& camera_transform = camera.load()->getTransform(glutGet(GLUT_ELAPSED_TIME));
    shader->setMat3("gCameraBasis", camera_transform.getBasis());
    shader->setVec3("gCameraOrigin", camera_transform.getOrigin());
    shader->setFloat("gProj[0]", (float)camera.load()->getProj(0));
    shader->setFloat("gProj[1]", (float)camera.load()->getProj(1));
    shader->setFloat("gProj[2]", (float)camera.load()->getProj(2));
    shader->setFloat("gProj[3]", (float)camera.load()->getProj(3));

    drawObjects();

    glutSwapBuffers();
}

static void reshape(int width, int height) {
    if (camera.load() == nullptr) return;
    camera.load()->reshape(width, height);
    glViewport(0, 0, width, height);
}

static void keyboard(unsigned char key, int, int) {
    if (camera.load() == nullptr) return;
    camera.load()->keyboard(key, 0);
}

static void keyboardUp(unsigned char key, int, int) {
    if (camera == nullptr) return;
    camera.load()->keyboard(key, 1);
}

static void mouse(int button, int state, int x, int y) {
    if (camera == nullptr) return;
    camera.load()->mouse(button, state, x, y);
}

static void motion(int x, int y) {
    if (camera == nullptr) return;
    camera.load()->motion(x, y);
}

static void timer(int) {
    glutPostRedisplay();
    glutTimerFunc(16, timer, 1);
}

static void close() {
    if (shader == nullptr) return;
    for (auto& obj: objs) {
        obj.second->deinit();
    }
    delete shader; shader = nullptr;
}

void OpenglViewer::open(const std::string &name, int width, int height) {
    if (glutGetWindow() != 0) {
        throw std::runtime_error("OpenglViewer is already opened");
    }

    int argc = 0; char **argv = 0;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(width, height);
    glutCreateWindow(name.c_str());
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glewInit();

    glutSetKeyRepeat(0);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutTimerFunc(16, timer, 1);
    glutCloseFunc(::close);

    shader = new ShaderProgram(shader_vert, shader_frag);
    shader->use();
    shader->setVec3("gLightDirection", pe_common::Vector3(1, -2, -3).normalized());

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.6, 0.85, 0.918, 1.);
    glClearDepth(1.);

    glutMainLoop();
}

void OpenglViewer::close() {
    if (glutGetWindow() != 0) {
        glutLeaveMainLoop();
    }
}

void OpenglViewer::setCamera(const pe_common::Vector3& position, PEReal yaw, PEReal pitch) {
    if (camera.load() == nullptr) {
        camera.store(new Camera);
        PEReal aspect = 1.0;
        if (glutGetWindow() != 0) {
            auto width = glutGet(GLUT_WINDOW_WIDTH);
            auto height = glutGet(GLUT_WINDOW_HEIGHT);
            aspect = (PEReal)width / height;
        }
        camera.load()->setProj(45., aspect, .1, 1000.);
    }
    camera.load()->setPosition(position);
    camera.load()->setYaw(yaw);
    camera.load()->setPitch(pitch);
}

int OpenglViewer::addMesh(const pe_common::Mesh& mesh, bool dynamic) {
    std::unique_lock<std::mutex> lock(mtx);
    objs.emplace_back(++max_id, new MeshRenderer(mesh, dynamic));
    return max_id;
}

static int findObj(int id, int type) {
    if (id < 0) return -1;
    int size = (int)objs.size();
    for (int idx = 0; idx < size; idx++) {
        if (objs[idx].first == id &&
            objs[idx].second->type() == type)
            return idx;
    }
    return -1;
}

//void OpenglViewer::qryObj(const ObjQryParam &param) {
//    std::unique_lock<std::mutex> lock(mtx);
//    if (param.action == ObjAction::OBJ_ADD) {
//        if (param.type == ObjType::OBJ_MESH) {
//            objs.emplace_back(++max_id, new MeshRenderer(param.mesh, param.dynamic));
//        } else if (param.type == ObjType::OBJ_LINE) {
//            objs.emplace_back(++max_id, new LineRenderer(param.start, param.end));
//        } else {
//            throw std::runtime_error("Unknown object type");
//        }
//    } else if (param.action == ObjAction::OBJ_UPDATE) {
//        int obj_idx = findObj(param.id, param.type);
//        if (obj_idx < 0) return;
//        if (param.type == ObjType::OBJ_MESH) {
//            auto mesh = dynamic_cast<MeshRenderer*>(objs[obj_idx].second);
//            if (param.attrib == ObjAttrib::MESH) {
//                mesh->updateMesh(param.mesh);
//            } else if (param.attrib == ObjAttrib::MESH_COLOR) {
//                mesh->setColor(param.color);
//            } else if (param.attrib == ObjAttrib::MESH_TRANSFORM) {
//                mesh->setTransform(param.transform);
//            } else {
//                throw std::runtime_error("Unknown object attribute");
//            }
//        } else if (param.type == ObjType::OBJ_LINE) {
//            auto line = dynamic_cast<LineRenderer*>(objs[obj_idx].second);
//            if (param.attrib == ObjAttrib::LINE) {
//                line->updateLine(param.start, param.end);
//            } else if (param.attrib == ObjAttrib::LINE_COLOR) {
//                line->setColor(param.color);
//            } else if (param.attrib == ObjAttrib::LINE_WIDTH) {
//                line->setWidth(param.width);
//            } else {
//                throw std::runtime_error("Unknown object attribute");
//            }
//        } else {
//            throw std::runtime_error("Unknown object type");
//        }
//    } else if (param.action == ObjAction::OBJ_DEL) {
//        int obj_idx = findObj(param.id, param.type);
//        if (obj_idx < 0) return;
//        objs[obj_idx].first = -1;
//    } else if (param.action == ObjAction::OBJ_CLEAR_ALL) {
//        for (auto& obj: objs) {
//            if (obj.second->type() == param.type) {
//                obj.first = -1;
//            }
//        }
//    } else {
//        throw std::runtime_error("Unknown object action");
//    }
//}

bool OpenglViewer::updateMesh(int id, const pe_common::Mesh& mesh) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_MESH);
    if (obj_idx < 0) return false;
    return dynamic_cast<MeshRenderer*>(objs[obj_idx].second)->updateMesh(mesh);
}

bool OpenglViewer::updateMeshTransform(int id, const pe_common::Transform& transform) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_MESH);
    if (obj_idx < 0) return false;
    dynamic_cast<MeshRenderer*>(objs[obj_idx].second)->setTransform(transform);
    return true;
}

bool OpenglViewer::updateMeshColor(int id, const pe_common::Vector3& color) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_MESH);
    if (obj_idx < 0) return false;
    dynamic_cast<MeshRenderer*>(objs[obj_idx].second)->setColor(color);
    return true;
}

void OpenglViewer::delMesh(int id) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_MESH);
    if (obj_idx < 0) return;
    objs[obj_idx].first = -1;
}

void OpenglViewer::clearMeshes() {
    std::unique_lock<std::mutex> lock(mtx);
    for (auto& mesh: objs) {
        if (mesh.second->type() == RenderType::R_MESH) {
            mesh.first = -1;
        }
    }
}

int OpenglViewer::addLine(const pe::Array<pe_common::Vector3>& points) {
    std::unique_lock<std::mutex> lock(mtx);
    objs.emplace_back(++max_id, new LineRenderer(points));
    return max_id;
}

bool OpenglViewer::updateLine(int id, const pe::Array<pe_common::Vector3>& points) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_LINE);
    if (obj_idx < 0) return false;
    dynamic_cast<LineRenderer*>(objs[obj_idx].second)->updateLine(points);
    return true;
}

bool OpenglViewer::updateLineWidth(int id, PEReal width) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_LINE);
    if (obj_idx < 0) return false;
    dynamic_cast<LineRenderer*>(objs[obj_idx].second)->setWidth(width);
    return true;
}

bool OpenglViewer::updateLineColor(int id, const pe_common::Vector3 &color) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_LINE);
    if (obj_idx < 0) return false;
    dynamic_cast<LineRenderer*>(objs[obj_idx].second)->setColor(color);
    return true;
}

void OpenglViewer::delLine(int id) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx = findObj(id, RenderType::R_LINE);
    if (obj_idx < 0) return;
    objs[obj_idx].first = -1;
}

void OpenglViewer::clearLines() {
    std::unique_lock<std::mutex> lock(mtx);
    for (auto& mesh: objs) {
        if (mesh.second->type() == RenderType::R_LINE) {
            mesh.first = -1;
        }
    }
}

OpenglViewer::~OpenglViewer() {
    close();
    for (auto& mesh: objs) {
        delete mesh.second;
    }
    for (auto& obj: objs) {
        delete obj.second;
    }
    delete camera;
}
