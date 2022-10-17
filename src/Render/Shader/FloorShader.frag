#version 330

in vec3 nearCoord;
in vec3 farCoord;
out vec4 FragColor;

uniform mat4 view;
uniform mat4 projection;
uniform float near;
uniform float far;

float computeDepth(vec3 pos) {
    vec4 clipCoord = projection * view * vec4(pos, 1.0);
    return clipCoord.z / clipCoord.w;
}

float computeLinearDepth(float depth) {
    return (2.0 * near * far) / (far + near - depth * (far - near)) / far;
}

void main() {
    float t = - nearCoord.z / (farCoord.z - nearCoord.z);
    vec3 planeCoord = nearCoord + t * (farCoord - nearCoord);
    float depth = computeDepth(planeCoord);
    float linearDepth = computeLinearDepth(depth * 2.0 - 1.0);
    float fading = max(0, (0.85 - linearDepth));
    FragColor = vec4(vec3(1.0, 1.0, 1.0) * float(floor(mod(planeCoord.x, 2)) == floor(mod(planeCoord.y, 2))), float(t > 0)) * fading;
    gl_FragDepth = depth;
}