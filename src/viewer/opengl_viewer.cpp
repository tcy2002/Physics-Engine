#include "opengl_viewer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <mutex>
#include <atomic>
#include "camera.h"
#include "mesh_renderer.h"
#include "shader_program.h"
#include "shader_vert.h"
#include "shader_frag.h"

using namespace pe_viewer;

static std::atomic<Camera*> camera = nullptr;
static ShaderProgram* program = nullptr;
static pe::Array<pe::Pair<int, MeshRenderer*>> meshes;
static int max_mesh_id = -1;
static std::mutex mtx_meshes;

static void display() {
    if (camera == nullptr || program == nullptr) return;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& camera_transform = camera.load()->getTransform(glutGet(GLUT_ELAPSED_TIME));
    program->setMat3("gCameraBasis", camera_transform.getBasis());
    program->setVec3("gCameraOrigin", camera_transform.getOrigin());
    program->setFloat("gProj[0]", (float)camera.load()->getProj(0));
    program->setFloat("gProj[1]", (float)camera.load()->getProj(1));
    program->setFloat("gProj[2]", (float)camera.load()->getProj(2));
    program->setFloat("gProj[3]", (float)camera.load()->getProj(3));

    std::unique_lock<std::mutex> lock(mtx_meshes);
    for (int i = (int)meshes.size() - 1; i >= 0; i--) {
        // update meshes
        if (meshes[i].first == -1) {
            delete meshes[i].second;
            meshes.erase(meshes.begin() + i);
            continue;
        } else if (!meshes[i].second->isInited()) {
            meshes[i].second->init();
        }
        // render meshes
        auto& transform = meshes[i].second->getTransform();
        program->setMat3("gWorldBasis", transform.getBasis());
        program->setVec3("gWorldOrigin", transform.getOrigin());
        program->setVec3("gColor", meshes[i].second->getColor());
        meshes[i].second->render();
    }
    lock.unlock();

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
    if (program == nullptr) return;
    for (auto& mesh: meshes) {
        mesh.second->deinit();
    }
    delete program; program = nullptr;
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

    program = new ShaderProgram(shader_vert, shader_frag);
    program->use();
    program->setFloat("gAmbientIntensity", 0.4f);
    program->setFloat("gDiffuseIntensity", 0.8f);
    program->setVec3("gLightDirection", pe_common::Vector3(1, -2, -3).normalized());

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
    std::unique_lock<std::mutex> lock(mtx_meshes);
    auto mr = new MeshRenderer(mesh, dynamic);
    meshes.emplace_back(++max_mesh_id, mr);
    return max_mesh_id;
}

static int findMesh(int id) {
    if (id < 0) return -1;
    int size = (int)meshes.size();
    for (int idx = 0; idx < size; idx++) {
        if (meshes[idx].first == id) return idx;
    }
    return -1;
}

bool OpenglViewer::updateMesh(int id, const pe_common::Mesh& mesh) {
    std::unique_lock<std::mutex> lock(mtx_meshes);
    int mesh_idx = findMesh(id);
    if (mesh_idx < 0) return false;
    return meshes[mesh_idx].second->updateMesh(mesh);
}

bool OpenglViewer::updateMeshTransform(int id, const pe_common::Transform& transform) {
    std::unique_lock<std::mutex> lock(mtx_meshes);
    int mesh_idx = findMesh(id);
    if (mesh_idx < 0) return false;
    meshes[mesh_idx].second->setTransform(transform);
    return true;
}

bool OpenglViewer::updateMeshColor(int id, const pe_common::Vector3& color) {
    std::unique_lock<std::mutex> lock(mtx_meshes);
    int mesh_idx = findMesh(id);
    if (mesh_idx < 0) return false;
    meshes[mesh_idx].second->setColor(color);
    return true;
}

void OpenglViewer::delMesh(int id) {
    std::unique_lock<std::mutex> lock(mtx_meshes);
    int mesh_idx = findMesh(id);
    if (mesh_idx < 0) return;
    meshes[mesh_idx].first = -1;
}

void OpenglViewer::clearMeshes() {
    std::unique_lock<std::mutex> lock(mtx_meshes);
    for (auto& mesh: meshes) {
        mesh.first = -1;
    }
}

OpenglViewer::~OpenglViewer() {
    close();
    for (auto& mesh: meshes) {
        delete mesh.second;
    }
    delete camera;
}
