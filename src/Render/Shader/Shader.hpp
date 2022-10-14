
#ifndef RENDERER_SHADER_HPP
#define RENDERER_SHADER_HPP

#include "glm/glm.hpp"
#include <string>

class Shader {
public:
    Shader(const std::string& vertex_shader_path, const std::string& fragment_shader_path);
    ~Shader();

    void Use();

    void SetInt(const std::string& name, int value);
    void SetFloat(const std::string& name, float value);
    void SetFloat(const std::string& name, int number, float value[]);
    void SetFloat(const std::string& name, const glm::vec2& value);
    void SetFloat(const std::string& name, const glm::vec3& value);
    void SetFloat(const std::string& name, const glm::vec4& value);
    void SetFloat(const std::string& name, const glm::mat2& value);
    void SetFloat(const std::string& name, const glm::mat3& value);
    void SetFloat(const std::string& name, const glm::mat4& value);

protected:
    unsigned int _program_id, _vertex_shader_id, _fragment_shader_id;
};

#endif
