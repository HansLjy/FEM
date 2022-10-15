#version 330 core

layout (location = 0) in vec3 aCoord;

out vec3 nearCoord;
out vec3 farCoord;

uniform mat4 view;
uniform mat4 projection;

vec3 Unproject(float x, float y, float z, mat4 view_inv, mat4 proj_inv) {
    vec4 origin = view_inv * proj_inv * vec4(x, y, z, 1.0);
    return origin.xyz / origin.w;
}

void main() {
    mat4 view_inv = inverse(view);
    mat4 proj_inv = inverse(projection);
    nearCoord = Unproject(aCoord.x, aCoord.y, -1.0, view_inv, proj_inv);
    farCoord = Unproject(aCoord.x, aCoord.y, 1.0, view_inv, proj_inv);
    gl_Position = vec4(aCoord, 1.0);
}