#version 330

in vec3 nearCoord;
in vec3 farCoord;
out vec4 FragColor;

uniform mat4 view;
uniform mat4 projection;

float computeDepth(vec3 pos) {
    vec4 clipCoord = projection * view * vec4(pos, 1.0);
    return clipCoord.z / clipCoord.w;
}

void main() {
    float t = - nearCoord.z / (farCoord.z - nearCoord.z);
    vec3 planeCoord = nearCoord + t * (farCoord - nearCoord);
    FragColor = vec4(vec3(1.0, 1.0, 1.0) * float(floor(mod(planeCoord.x, 2)) == floor(mod(planeCoord.y, 2))), float(t > 0));
//    FragColor = vec4(vec3(1.0, 1.0, 1.0), 1.0);
    gl_FragDepth = computeDepth(planeCoord);
}