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

using namespace simple_viewer;

//// camera: might be set by two threads at the same time
static std::atomic<Camera*> camera = nullptr;

//// shader
static ShaderProgram* shader = nullptr;

//// object
static std::vector<std::pair<int, Renderer*>> objs;
static int max_id = -1;
static std::mutex mtx;

static void drawMesh(MeshRenderer* mesh) {
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
            delete objs[i].second;
            objs.erase(objs.begin() + i);
            continue;
        } else if (!objs[i].second->isInited()) {
            objs[i].second->init(1, 2);
        }

        // render object
        auto& transform = objs[i].second->getTransform();
        shader->setMat3("gWorldBasis", transform.getBasis());
        shader->setVec3("gWorldOrigin", transform.getOrigin());
        shader->setVec3("gColor", objs[i].second->getColor());
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

// some unknown bug in glut: when key is pressed, and the window
// is shutdown before key is released, the next time open the
// window, glut will assume the key is still pressed whatever
// the real state is. no solution found yet.

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
    delete shader; shader = nullptr;
    std::unique_lock<std::mutex> lock(mtx);
    for (auto& obj: objs) {
        obj.second->deinit();
    }
    // reset glut state
    glutSetKeyRepeat(1);
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
    shader->setVec3("gLightDirection", Vector3(1, -2, -3).normalized());

    if (camera.load() != nullptr) {
        camera.load()->reset(glutGet(GLUT_ELAPSED_TIME));
    }

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

void OpenglViewer::setCamera(const Vector3& position, float yaw, float pitch) {
    if (camera.load() == nullptr) {
        camera.store(new Camera);
        float aspect = 1.0;
        if (glutGetWindow() != 0) {
            auto width = glutGet(GLUT_WINDOW_WIDTH);
            auto height = glutGet(GLUT_WINDOW_HEIGHT);
            aspect = (float)width / (float)height;
        }
        camera.load()->setProj(45., aspect, .1, 1000.);
    }
    camera.load()->setPosition(position);
    camera.load()->setYaw(yaw);
    camera.load()->setPitch(pitch);
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

int OpenglViewer::addObj(const ObjInitParam &param) {
    std::unique_lock<std::mutex> lock(mtx);
    switch (param.type) {
        case ObjType::OBJ_MESH:
            objs.emplace_back(++max_id, new MeshRenderer(param.mesh, param.dynamic));
            break;
        case ObjType::OBJ_LINE:
            objs.emplace_back(++max_id, new LineRenderer(param.line, param.dynamic));
            break;
        default:
            throw std::runtime_error("Unknown object type");
    }
    return max_id;
}

bool OpenglViewer::updateObj(const ObjUpdateParam &param) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx;
    if (param.act_type != OBJ_CLEAR_ALL_TYPE) {
        obj_idx = findObj(param.obj_id, param.obj_type);
        if (obj_idx < 0) return false;
    }

    switch (param.act_type) {
        case OBJ_UPDATE_TRANSFORM:
            objs[obj_idx].second->setTransform(param.transform);
            return true;
        case OBJ_UPDATE_COLOR:
            objs[obj_idx].second->setColor(param.color);
            return true;
        case OBJ_UPDATE_MESH:
            return dynamic_cast<MeshRenderer*>(objs[obj_idx].second)->updateMesh(param.mesh);
        case OBJ_UPDATE_LINE:
            return dynamic_cast<LineRenderer*>(objs[obj_idx].second)->updateLine(param.line);
        case OBJ_UPDATE_LINE_WIDTH:
            dynamic_cast<LineRenderer*>(objs[obj_idx].second)->setWidth(param.line_width);
            return true;
        case OBJ_DEL:
            objs[obj_idx].first = -1;
            return true;
        case OBJ_CLEAR_ALL_TYPE:
            for (auto& obj: objs) {
                if (obj.second->type() == param.obj_type) {
                    obj.first = -1;
                }
            }
            return true;
        default:
            throw std::runtime_error("Unknown update command type");
    }
}

OpenglViewer::~OpenglViewer() {
    close();
    for (auto& obj: objs) {
        delete obj.second;
    }
    delete camera.load();
}
