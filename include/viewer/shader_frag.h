#pragma once

#define shader_frag \
"#version 450\n"\
"\n"\
"in VS_OUT {\n"\
"    vec3 position;\n"\
"    vec3 normal;\n"\
"} fs_in;\n"\
"\n"\
"out vec4 FragColor;\n"\
"\n"\
"\n"\
"uniform float gAmbientIntensity;\n"\
"uniform float gDiffuseIntensity;\n"\
"uniform vec3 gLightDirection;\n"\
"uniform vec3 gColor;\n"\
"\n"\
"void main() {\n"\
"    vec3 ambient = gColor * gAmbientIntensity;\n"\
"    vec3 diffuse = gColor * gDiffuseIntensity * clamp(dot(fs_in.normal, -gLightDirection), 0, 1);\n"\
"    FragColor = vec4(ambient + diffuse, 1.0f);\n"\
"    //FragColor = vec4(gColor, 1.0f);\n"\
"}"
