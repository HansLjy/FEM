#version 330 core

layout (location = 0) in vec3 aCoord;   // we assume this is the world coordination
layout (location = 1) in vec3 aNormal;

out vec3 WorldCoord;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {

    gl_Position = projection * view * model * vec4(aCoord, 1.0);
    // gl_Position = vec4(aCoord, 1.0);
    WorldCoord = aCoord;
    Normal = aNormal;
}