#include "viewer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include "shader_vert.h"
#include "shader_frag.h"

using namespace pe_ui;

void displayWrapper() {
    Viewer::initViewerInstance().display();
}

void reshapeWrapper(int width, int height) {
    Viewer::initViewerInstance().reshape(width, height);
}

void keyboardWrapper(unsigned char key, int x, int y) {
    Viewer::initViewerInstance().keyboard(key, x, y);
}

void keyboardUpWrapper(unsigned char key, int x, int y) {
    Viewer::initViewerInstance().keyboardUp(key, x, y);
}

void mouseWrapper(int button, int state, int x, int y) {
    Viewer::initViewerInstance().mouse(button, state, x, y);
}

void motionWrapper(int x, int y) {
    Viewer::initViewerInstance().motion(x, y);
}

void timer(int) {
    glutPostRedisplay();
    glutTimerFunc(16, timer, 1);
}

void Viewer::display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& camera_transform = _camera->getTransform(glutGet(GLUT_ELAPSED_TIME));
    _program->setMat3("gCameraBasis", camera_transform.getBasis());
    _program->setVec3("gCameraOrigin", camera_transform.getOrigin());
    _program->setFloat("gProj[0]", (float)_camera->getProj(0));
    _program->setFloat("gProj[1]", (float)_camera->getProj(1));
    _program->setFloat("gProj[2]", (float)_camera->getProj(2));
    _program->setFloat("gProj[3]", (float)_camera->getProj(3));

    for (auto &mesh: _meshes) {
        auto& transform = mesh->getTransform();
        _program->setMat3("gWorldBasis", transform.getBasis());
        _program->setVec3("gWorldOrigin", transform.getOrigin());
        _program->setVec3("gColor", mesh->getColor());
        mesh->render();
    }

    glutSwapBuffers();
}

void Viewer::reshape(int width, int height) {
    _camera->reshape(width, height);
    glViewport(0, 0, width, height);
}

void Viewer::keyboard(unsigned char key, int, int) {
    _camera->keyboard(key, 0);
}

void Viewer::keyboardUp(unsigned char key, int, int) {
    _camera->keyboard(key, 1);
}

void Viewer::mouse(int button, int state, int x, int y) {
    _camera->mouse(button, state, x, y);
}

void Viewer::motion(int x, int y) {
    _camera->motion(x, y);
}

Viewer::Viewer(int width, int height) {
    int argc = 0;
    char **argv = nullptr;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("Physics Engine Viewer");
    glewInit();

    glutSetKeyRepeat(0);
    glutDisplayFunc(displayWrapper);
    glutReshapeFunc(reshapeWrapper);
    glutKeyboardFunc(keyboardWrapper);
    glutKeyboardUpFunc(keyboardUpWrapper);
    glutMouseFunc(mouseWrapper);
    glutMotionFunc(motionWrapper);
    glutTimerFunc(16, timer, 1);

    _camera = new Camera;
    _camera->setProj(45., 1. * width / height, 0.1, 1000.);
    _program = new ShaderProgram(shader_vert, shader_frag);
}

Viewer::~Viewer() {
    delete _camera;
    delete _program;
    for (auto &mesh: _meshes) {
        delete mesh;
    }
}

Viewer &Viewer::initViewerInstance(int width, int height) {
    static Viewer instance(width, height);
    return instance;
}

void Viewer::setCamera(const pe_cg::Vector3& position, real yaw, real pitch) {
    _camera->setPosition(position);
    _camera->setYaw(yaw);
    _camera->setPitch(pitch);
}

void Viewer::addMesh(const pe_cg::Mesh& mesh,
                     const pe_cg::Transform& transform,
                     const pe_cg::Vector3& color) {
    _meshes.push_back(new MeshRenderer(mesh));
    _meshes.back()->setTransform(transform);
    _meshes.back()->setColor(color);
}

void Viewer::run() {
    _program->use();
    _program->setFloat("gAmbientIntensity", 0.4f);
    _program->setFloat("gDiffuseIntensity", 0.8f);
    _program->setVec3("gLightDirection", pe_cg::Vector3(1, -2, -3).normalized());
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.6, 0.85, 0.918, 1.);
    glClearDepth(1.);
    glutMainLoop();
}
