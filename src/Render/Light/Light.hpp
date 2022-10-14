#ifndef LIGHT_HPP
#define LIGHT_HPP

#include "glm/glm.hpp"

struct Light {
    Light(const glm::vec3& position,
          const glm::vec3& ambient,
          const glm::vec3& diffuse,
          const glm::vec3& specular,
          float Kc, float Kl, float Kq)  : 
    _position(position), _ambient(ambient),
    _diffuse(diffuse), _specular(specular),
    _Kc(Kc), _Kl(Kl), _Kq(Kq) {}

    glm::vec3 _position;
    glm::vec3 _ambient;
    glm::vec3 _diffuse;
    glm::vec3 _specular;

    float _Kc, _Kl, _Kq;
};


#endif