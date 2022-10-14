#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include "glm/glm.hpp"

struct Material {

    Material(const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float shininess):
        _ambient(ambient), _diffuse(diffuse), _specular(specular), _shininess(shininess) {}

    glm::vec3 _ambient;
    glm::vec3 _diffuse;
    glm::vec3 _specular;

    float _shininess;
};

#endif