#pragma once

#define shader_vert \
"#version 450\n"\
"\n"\
"layout (location = 1) in vec3 gPosition;\n"\
"layout (location = 2) in vec3 gNormal;\n"\
"\n"\
"out VS_OUT {\n"\
"    vec3 position;\n"\
"    vec3 normal;\n"\
"} vs_out;\n"\
"\n"\
"uniform mat3 gWorldBasis;\n"\
"uniform vec3 gWorldOrigin;\n"\
"uniform mat3 gCameraBasis;\n"\
"uniform vec3 gCameraOrigin;\n"\
"uniform float gProj[4];\n"\
"\n"\
"void main() {\n"\
"    vec3 worldPos = gWorldBasis * gPosition + gWorldOrigin;\n"\
"    vs_out.position = worldPos;\n"\
"    vs_out.normal = gWorldBasis * gNormal;\n"\
"    vec3 cameraPos = transpose(gCameraBasis) * (worldPos - gCameraOrigin);\n"\
"    gl_Position = vec4(gProj[0] * cameraPos.x, gProj[1] * cameraPos.y,\n"\
"                       gProj[2] * cameraPos.z + gProj[3], -cameraPos.z);\n"\
"}\n"
