#version 330 core

layout (location = 0) in vec3 aCoord;   // we assume this is the world coordination
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTextureCoord;

out vec3 WorldCoord;
out vec3 Normal;
out vec2 TextureCoord;

uniform mat4 view;
uniform mat4 projection;
uniform mat3 rotation;
uniform vec3 shift;

void main() {

    gl_Position = projection * view * vec4(rotation * aCoord + shift, 1.0);
    // gl_Position = vec4(aCoord, 1.0);
    WorldCoord = rotation * aCoord + shift;
    Normal = rotation * aNormal;
    TextureCoord = aTextureCoord;
}