#include "Shader.hpp"
#include <cstddef>
#include <cstdlib>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include "glad/glad.h"
#include "glm/gtc/type_ptr.hpp"
#include "spdlog/spdlog.h"

Shader::Shader(const std::string& vertex_shader_file, const std::string& fragment_shader_file) {
    std::fstream vertex_shader_stream(vertex_shader_file);
    std::fstream fragment_shader_stream(fragment_shader_file);

    std::stringstream vertex_shader_sstream, fragment_shader_sstream;
    vertex_shader_sstream << vertex_shader_stream.rdbuf();
    fragment_shader_sstream << fragment_shader_stream.rdbuf();

    vertex_shader_stream.close();
    fragment_shader_stream.close();

    auto vertex_shader_program = vertex_shader_sstream.str();
    auto fragment_shader_program = fragment_shader_sstream.str();

    auto vertex_shader_pointer = vertex_shader_program.c_str();
    auto fragment_shader_pointer = fragment_shader_program.c_str();

    _vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    _fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    int success;
    char info[512];

    glShaderSource(_vertex_shader_id, 1, &vertex_shader_pointer, NULL);
    glCompileShader(_vertex_shader_id);
    glGetShaderiv(_vertex_shader_id, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(_vertex_shader_id, 512, NULL, info);
        spdlog::error("Vertex shader compilation error: {}", info);
        exit(-1);
    }

    glShaderSource(_fragment_shader_id, 1, &fragment_shader_pointer, NULL);
    glCompileShader(_fragment_shader_id);
    glGetShaderiv(_fragment_shader_id, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(_fragment_shader_id, 512, NULL, info);
        spdlog::error("Fragment shader compilation error: {}", info);
        exit(-1);
    }

    _program_id = glCreateProgram();
    glAttachShader(_program_id, _vertex_shader_id);
    glAttachShader(_program_id, _fragment_shader_id);
    glLinkProgram(_program_id);

    glGetProgramiv(_program_id, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(_program_id, 512, NULL, info);
        spdlog::error("Program linking error: {}", info);
        exit(-1);
    }
}

Shader::~Shader() {
    glDeleteShader(_vertex_shader_id);
    glDeleteShader(_fragment_shader_id);
    glDeleteProgram(_program_id);
}

void Shader::Use() {
    glUseProgram(_program_id);
}

void Shader::SetInt(const std::string &name, int value) {
    glUniform1i(glGetUniformLocation(_program_id, name.c_str()), value);
}

void Shader::SetFloat(const std::string &name, float value) {
    glUniform1f(glGetUniformLocation(_program_id, name.c_str()), value);
}

void Shader::SetFloat(const std::string& name, int number, float value[]) {
    switch (number) {
        case 2:
            glUniform2f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1]);
            break;
        case 3:
            glUniform3f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1], value[2]);
            break;
        case 4:
            glUniform4f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1], value[2], value[3]);
            break;
        default:
            spdlog::error("GLSL do not support vector of size other than 2, 3, 4!");
            exit(-1);
    }
}

void Shader::SetFloat(const std::string& name, const glm::vec2& value) {
    glUniform2f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1]);
}

void Shader::SetFloat(const std::string& name, const glm::vec3& value) {
    glUniform3f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1], value[2]);
}

void Shader::SetFloat(const std::string& name, const glm::vec4& value) {
    glUniform4f(glGetUniformLocation(_program_id, name.c_str()), value[0], value[1], value[2], value[3]);
}

void Shader::SetFloat(const std::string& name, const glm::mat2& value) {
    glUniformMatrix2fv(glGetUniformLocation(_program_id, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::SetFloat(const std::string& name, const glm::mat3& value) {
    glUniformMatrix3fv(glGetUniformLocation(_program_id, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::SetFloat(const std::string& name, const glm::mat4& value) {
    glUniformMatrix4fv(glGetUniformLocation(_program_id, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}