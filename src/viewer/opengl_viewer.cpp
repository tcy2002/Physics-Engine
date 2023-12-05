#include "opengl_viewer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <mutex>
#include "camera.h"
#include "renderer.h"
#include "shader_program.h"
#include "shader_vert.h"
#include "shader_frag.h"

namespace simple_viewer {

//// one global lock is enough
static std::mutex mtx;

//// camera
static Camera* camera = nullptr;

//// shader
static ShaderProgram* shader = nullptr;

//// object
static std::vector<std::pair<int, Renderer*>> objs;
static int max_id = -1;

//// axis
static LineRenderer* axes[3] = { // NOLINT
    new LineRenderer({Vector3(0, 0, 0), Vector3(1, 0, 0)}),
    new LineRenderer({Vector3(0, 0, 0), Vector3(0, 1, 0)}),
    new LineRenderer({Vector3(0, 0, 0), Vector3(0, 0, 1)})
};
static Vector3 axis_color[3] = { // NOLINT
    Vector3(0, 0, 1), Vector3(0, 1, 0), Vector3(1, 0, 0)
};
static bool show_axis = true;

static void drawAxis(int axis) {
    shader->setFloat("gAmbientIntensity", 1.0f);
    shader->setFloat("gDiffuseIntensity", 0);
    shader->setMat3("gWorldBasis", Matrix3::identity());
    shader->setVec3("gWorldOrigin", Vector3::zeros());
    shader->setVec3("gColor", axis_color[axis]);

    // init axis
    if (!axes[axis]->isInited()) {
        axes[axis]->init(1, 2);
    }

    // render axis
    axes[axis]->render();
}

static void drawMesh(MeshRenderer* mesh) {
    auto& transform = mesh->getTransform();
    shader->setMat3("gWorldBasis", transform.getBasis());
    shader->setVec3("gWorldOrigin", transform.getOrigin());
    shader->setVec3("gColor", mesh->getColor());
    shader->setFloat("gAmbientIntensity", 0.5f);
    shader->setFloat("gDiffuseIntensity", 0.8f);

    // init mesh
    if (!mesh->isInited()) {
        mesh->init(1, 2);
    }

    // render mesh
    mesh->render();
}

static void drawCube(CubeRenderer* cube) {
    auto& transform = cube->getTransform();
    shader->setMat3("gWorldBasis", transform.getBasis());
    shader->setVec3("gWorldOrigin", transform.getOrigin());
    shader->setVec3("gColor", cube->getColor());
    shader->setFloat("gAmbientIntensity", 0.5f);
    shader->setFloat("gDiffuseIntensity", 0.8f);

    // init mesh
    if (!cube->isInited()) {
        cube->init(1, 2);
    }

    // render mesh
    cube->render();
}

static void drawCylinder(CylinderRenderer* cyl) {
    auto& transform = cyl->getTransform();
    shader->setMat3("gWorldBasis", transform.getBasis());
    shader->setVec3("gWorldOrigin", transform.getOrigin());
    shader->setVec3("gColor", cyl->getColor());
    shader->setFloat("gAmbientIntensity", 0.5f);
    shader->setFloat("gDiffuseIntensity", 0.8f);

    // init mesh
    if (!cyl->isInited()) {
        cyl->init(1, 2);
    }

    // render mesh
    cyl->render();
}

static void drawLine(LineRenderer* line) {
    auto& transform = line->getTransform();
    shader->setMat3("gWorldBasis", transform.getBasis());
    shader->setVec3("gWorldOrigin", transform.getOrigin());
    shader->setVec3("gColor", line->getColor());
    shader->setFloat("gAmbientIntensity", 1.0f);
    shader->setFloat("gDiffuseIntensity", 0);

    // init line
    if (!line->isInited()) {
        line->init(1, 2);
    }

    // render line
    line->render();
}

static void drawObjects() {
    std::unique_lock<std::mutex> lock(mtx);
    for (int i = (int)objs.size() - 1; i >= 0; i--) {
        // check if object is deleted
        if (objs[i].first == -1) {
            objs[i].second->deinit();
            delete objs[i].second;
            objs.erase(objs.begin() + i);
            continue;
        }

        // render object
        if (objs[i].second->type() == RenderType::R_MESH) {
            drawMesh(dynamic_cast<MeshRenderer*>(objs[i].second));
        } else if (objs[i].second->type() == RenderType::R_CUBE) {
            drawCube(dynamic_cast<CubeRenderer*>(objs[i].second));
        } else if (objs[i].second->type() == RenderType::R_CYLINDER) {
            drawCylinder(dynamic_cast<CylinderRenderer*>(objs[i].second));
        } else if (objs[i].second->type() == RenderType::R_LINE) {
            drawLine(dynamic_cast<LineRenderer*>(objs[i].second));
        }
    }
}

static void display() {
    if (camera == nullptr || shader == nullptr) return;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& camera_transform = camera->getTransform(glutGet(GLUT_ELAPSED_TIME));
    shader->setMat3("gCameraBasis", camera_transform.getBasis());
    shader->setVec3("gCameraOrigin", camera_transform.getOrigin());
    shader->setFloat("gProj[0]", (float)camera->getProj(0));
    shader->setFloat("gProj[1]", (float)camera->getProj(1));
    shader->setFloat("gProj[2]", (float)camera->getProj(2));
    shader->setFloat("gProj[3]", (float)camera->getProj(3));

    drawObjects();
    if (show_axis) {
        drawAxis(0);
        drawAxis(1);
        drawAxis(2);
    }

    glutSwapBuffers();
}

static void reshape(int width, int height) {
    if (camera == nullptr) return;
    camera->reshape(width, height);
    glViewport(0, 0, width, height);
}

static void keyboard(unsigned char key, int, int) {
    if (camera == nullptr) return;
    camera->keyboard(key, 0);
}

static void keyboardUp(unsigned char key, int, int) {
    if (camera == nullptr) return;
    camera->keyboard(key, 1);
}

static void mouse(int button, int state, int x, int y) {
    if (camera == nullptr) return;
    camera->mouse(button, state, x, y);
}

static void motion(int x, int y) {
    if (camera == nullptr) return;
    camera->motion(x, y);
}

static void timer(int) {
    glutPostRedisplay();
    glutTimerFunc(16, timer, 1);
}

static void close_() {
    if (shader == nullptr) return;
    delete shader; shader = nullptr;
    // deinit all objects
    std::unique_lock<std::mutex> lock(mtx);
    // reset camera state
    if (camera != nullptr) {
        camera->reset();
    }
    // deinit objects
    for (auto& obj: objs) {
        obj.second->deinit();
    }
    // deinit axes
    axes[0]->deinit();
    axes[1]->deinit();
    axes[2]->deinit();
}

void open(const std::string &name, int width, int height) {
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
    glutCloseFunc(close_);

    // init a globally used shader
    shader = new ShaderProgram(shader_vert, shader_frag);
    shader->use();
    shader->setVec3("gLightDirection", Vector3(1, -2, -3).normalized());

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.6, 0.85, 0.918, 1.);
    glClearDepth(1.);

    glutMainLoop();
}

void close() {
    if (glutGetWindow() != 0) {
        glutLeaveMainLoop(); // leave is ok (will trigger ::close())
    }
}

void setCamera(const Vector3& position, float yaw, float pitch) {
    if (camera == nullptr) {
        // in case that 2 threads are initializing camera at the same time
        std::unique_lock<std::mutex> lock(mtx);
        camera = new Camera;
        lock.unlock();
        // initialize projection of the camera
        float aspect = 1.0;
        if (glutGetWindow() != 0) {
            auto width = glutGet(GLUT_WINDOW_WIDTH);
            auto height = glutGet(GLUT_WINDOW_HEIGHT);
            aspect = (float)width / (float)height;
        }
        camera->setProj(45., aspect, .1, 1000.);
    }
    camera->setPosition(position);
    camera->setYaw(yaw);
    camera->setPitch(pitch);
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

int addObj(const ObjInitParam &param) {
    std::unique_lock<std::mutex> lock(mtx);
    switch (param.type) {
        case ObjType::OBJ_MESH:
            objs.emplace_back(++max_id, new MeshRenderer(param.mesh, param.dynamic));
            break;
        case ObjType::OBJ_CUBE:
            objs.emplace_back(++max_id, new CubeRenderer(param.size, param.dynamic));
            break;
        case ObjType::OBJ_CYLINDER:
            objs.emplace_back(++max_id, new CylinderRenderer((float)param.size.x, (float)param.size.y, param.dynamic));
            break;
        case ObjType::OBJ_LINE:
            objs.emplace_back(++max_id, new LineRenderer(param.line, param.dynamic));
            break;
        default:
            throw std::runtime_error("Unknown object type");
    }
    return max_id;
}

bool updateObj(const ObjUpdateParam &param) {
    std::unique_lock<std::mutex> lock(mtx);
    int obj_idx;
    if (param.act_type != OBJ_CLEAR_ALL_TYPE && param.act_type != OBJ_CLEAR_ALL) {
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
        case OBJ_CLEAR_ALL:
            for (auto& obj: objs) {
                obj.first = -1;
            }
            return true;
        default:
            throw std::runtime_error("Unknown update command type");
    }
}

void showAxis(bool show) {
    std::unique_lock<std::mutex> lock(mtx);
    show_axis = show;
}

} // namespace simple_viewer