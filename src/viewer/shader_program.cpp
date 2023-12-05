#include "shader_program.h"

#include "GL/glew.h"

using namespace simple_viewer;

ShaderProgram::ShaderProgram(const char* vert_path, const char* frag_path) {
    _program_id = glCreateProgram();

    int vert_id = createShader(vert_path, GL_VERTEX_SHADER);
    glAttachShader(_program_id, vert_id);
    glDeleteShader(vert_id);

    int frag_id = createShader(frag_path, GL_FRAGMENT_SHADER);
    glAttachShader(_program_id, frag_id);
    glDeleteShader(frag_id);

    linkProgramAndCheck(_program_id);
}

ShaderProgram::~ShaderProgram() {
    glDeleteProgram(_program_id);
}

int ShaderProgram::createShader(const char* buf, int type) {
    int shader = glCreateShader(type);
    glShaderSource(shader, 1, &buf, nullptr);
    compileShaderAndCheck(shader);
    return shader;
}

void ShaderProgram::compileShaderAndCheck(int shader) {
    int success;
    char infoLog[1024];
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
        std::cout << "Shader Compile Error: \n" << infoLog << std::endl;
        exit(EXIT_FAILURE);
    }
}

void ShaderProgram::linkProgramAndCheck(int program) {
    int success;
    char infoLog[1024];
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 1024, nullptr, infoLog);
        std::cout << "Program Compile Error: \n" << infoLog << std::endl;
        exit(EXIT_FAILURE);
    }
}

void ShaderProgram::use() const {
    glUseProgram(_program_id);
}

void ShaderProgram::setBool(const char* name, bool value) const {
    glUniform1i(glGetUniformLocation(_program_id, name), (int)value);
}

void ShaderProgram::setInt(const char* name, int value) const {
    glUniform1i(glGetUniformLocation(_program_id, name), value);
}

void ShaderProgram::setFloat(const char* name, float value) const {
    glUniform1f(glGetUniformLocation(_program_id, name), value);
}

void ShaderProgram::setVec3(const char* name, const Vector3& value) const {
    glUniform3f(glGetUniformLocation(_program_id, name), (float)value.x, (float)value.y, (float)value.z);
}

void ShaderProgram::setMat3(const char* name, const Matrix3& value) const {
    static float buf[9];
    buf[0] = (float)value[0][0]; buf[1] = (float)value[1][0]; buf[2] = (float)value[2][0];
    buf[3] = (float)value[0][1]; buf[4] = (float)value[1][1]; buf[5] = (float)value[2][1];
    buf[6] = (float)value[0][2]; buf[7] = (float)value[1][2]; buf[8] = (float)value[2][2];
    glUniformMatrix3fv(glGetUniformLocation(_program_id, name), 1, GL_FALSE, buf);
}
