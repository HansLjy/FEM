#version 330 core

in vec2 TextureCoord;
in vec3 WorldCoord;
in vec3 Normal;
out vec4 FragColor;

uniform vec3 eye;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;

    float shininess;
};

uniform Material material;

struct Light {
    vec3 source;        // position of light source

    vec3 ambient;       // color of ambient light
    vec3 diffuse;       // color of diffusion light
    vec3 specular;      // color of specular light
};

uniform Light light;

uniform int useTexture;
uniform sampler2D ourTexture;

void main() {
    if (useTexture != 0) {
        vec3 texture_color = texture(ourTexture, TextureCoord).xyz;
        vec3 ambient = light.ambient * texture_color;

        vec3 norm = normalize(Normal);
        vec3 light_dir = normalize(light.source - WorldCoord);
        vec3 eye_dir = normalize(eye - WorldCoord);

        float diffuse_strength = max(dot(norm, light_dir), 0);
        vec3 diffuse = diffuse_strength * texture_color * light.diffuse;

        float specular_strength = max(dot(eye_dir, reflect(-light_dir, norm)), 0);
        vec3 specular = specular_strength * texture_color * light.specular;

        FragColor = vec4(diffuse + ambient + specular, 1.0);
    } else {
        vec3 ambient = light.ambient * material.ambient;
        
        vec3 norm = normalize(Normal);
        vec3 light_dir = normalize(light.source - WorldCoord);
        vec3 eye_dir = normalize(eye - WorldCoord);

        float diffuse_strength = max(dot(norm, light_dir), 0);
        vec3 diffuse = diffuse_strength * material.diffuse * light.diffuse;

        float specular_strength = max(dot(eye_dir, reflect(-light_dir, norm)), 0);
        vec3 specular = specular_strength * material.specular * light.specular;

        FragColor = vec4(diffuse + ambient + specular, 1.0);
    }
}