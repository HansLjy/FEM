#include "GUI.hpp"
#include "Camera/Camera.hpp"
#include "EigenAll.h"
#include "GLFW/glfw3.h"
#include "Light/Light.hpp"
#include "Scene/Scene.hpp"
#include "Shader/Shader.hpp"
#include "RenderObject/RendererObject.hpp"
#include "Light/Material.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <GL/gl.h>
#include <cstddef>
#include <cstdlib>
#include "spdlog/spdlog.h"
#include <string>
#include <iostream>

GUI::GUI(const std::string &title, int width, int height, int opengl_major_version, int opengl_minor_version)
         : _title(title), _width(width), _height(height),
           _major_version(opengl_major_version),
           _minor_version(opengl_minor_version){
    theGUI = this;
    _vs_path = SHADER_PATH "/ObjectShader.vert";
    _fs_path = SHADER_PATH "/ObjectShader.frag";
}

GUI::~GUI() {
    delete _camera;
    delete _light;
}

void GUI::SetCamera(const glm::vec3 &eye, const glm::vec3 &front, const glm::vec3 &up) {
    delete _camera;
    _camera = new Camera(eye, front, up);
}

void
GUI::SetLight(const glm::vec3 &position, const glm::vec3 &ambient, const glm::vec3 &diffuse, const glm::vec3 &specular,
              float Kc, float Kl, float Kq) {
    delete _light;
    _light = new Light(position, ambient, diffuse, specular, Kc, Kl, Kq);
}


Material material(
        glm::vec3(1.0f, 0.5f, 0.31f),
        glm::vec3(1.0f, 0.5f, 0.31f),
        glm::vec3(0.5f, 0.5f, 0.5f),
        32.0f
);

float floor_vertices [] = {
        1.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f, -1.0f, 1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, -1.0f, 0.0f
};

void GUI::MainLoop() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, _major_version);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, _minor_version);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    _window = glfwCreateWindow(_width, _height, _title.c_str(), NULL, NULL);
    if (_window == NULL) {
        spdlog::error("Failure in creating window");
        glfwTerminate();
        exit(-1);
    }
    glfwMakeContextCurrent(_window);
    glfwSetFramebufferSizeCallback(_window, GUI::ResizeCB);
    glfwSetScrollCallback(_window, GUI::ScrollCB);
    glfwSetCursorPosCallback(_window, GUI::MouseMovementCB);
    glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
        glfwTerminate();
        exit(-1);
    }

    Shader object_shader(_vs_path, _fs_path);
    Shader floor_shader(
        SHADER_PATH "/FloorShader.vert",
        SHADER_PATH "/FloorShader.frag"
    );

    float last_frame = static_cast<float>(glfwGetTime());
    float cur_frame;


    /* initialize floor */
    glGenVertexArrays(1, &_floor_VAO);
    {
        unsigned int VBO;
        glGenBuffers(1, &VBO);

        glBindVertexArray(_floor_VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof (float) * 18, floor_vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        glDeleteBuffers(1, &VBO);
    }


    Scene scene;
    InitializeScene(scene);

    while (!glfwWindowShouldClose(_window)) {
        cur_frame = static_cast<float>(glfwGetTime());
        ProcessKeyboard(cur_frame - last_frame);
        last_frame = cur_frame;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        object_shader.Use();

        glm::mat4 view = _camera->GetViewMatrix();
        glm::mat4 projection = _camera->GetProjectionMatrix((float)_width / (float)_height);

        object_shader.SetFloat("view", view);
        object_shader.SetFloat("projection", projection);

        object_shader.SetFloat("eye", _camera->_eye);

        object_shader.SetFloat("light.source", _camera->_eye);       // the light always comes from your eyes
        object_shader.SetFloat("light.ambient", _light->_ambient);
        object_shader.SetFloat("light.diffuse", _light->_diffuse);
        object_shader.SetFloat("light.specular", _light->_specular);

        object_shader.SetFloat("material.ambient", material._ambient);
        object_shader.SetFloat("material.diffuse", material._diffuse);
        object_shader.SetFloat("material.specular", material._specular);
        object_shader.SetFloat("material.shininess", material._shininess);

        Processing(scene);

        scene.Draw();

        // TODO draw floor
        floor_shader.Use();
        floor_shader.SetFloat("view", view);
        floor_shader.SetFloat("projection", projection);

        floor_shader.SetFloat("near", _camera->_near);
        floor_shader.SetFloat("far", _camera->_far);
        glBindVertexArray(_floor_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        glfwSwapBuffers(_window);
        glfwPollEvents();
    }

    glDeleteBuffers(1, &_floor_VAO);
    glfwTerminate();
}



void GUI::ResizeCB(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

#define PRESS(key) glfwGetKey(_window, GLFW_KEY_##key) == GLFW_PRESS
void GUI::ProcessKeyboard(float duration) {
    if (PRESS(W)) {
        _camera->CameraMovement(CameraMovementType::kForward, duration);
    }
    if (PRESS(S)) {
        _camera->CameraMovement(CameraMovementType::kBackward, duration);
    }
    if (PRESS(A)) {
        _camera->CameraMovement(CameraMovementType::kLeftward, duration);
    }
    if (PRESS(D)) {
        _camera->CameraMovement(CameraMovementType::kRightward, duration);
    }
    if (PRESS(Q)) {
        _camera->CameraMovement(CameraMovementType::kUpward, duration);
    }
    if (PRESS(E)) {
        _camera->CameraMovement(CameraMovementType::kDownward, duration);
    }
    if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(_window, true);
    }
}
#undef PRESS

void GUI::MouseMovementCB(GLFWwindow *window, double x_pos, double y_pos) {
    static bool is_first = true;
    static float last_x, last_y;
    if (is_first) {
        is_first = false;
        last_x = x_pos;
        last_y = y_pos;
        return;
    }

    GetGUI()->_camera->CameraRotation(x_pos - last_x, last_y - y_pos);
    last_x = x_pos;
    last_y = y_pos;
}

void GUI::ScrollCB(GLFWwindow *window, double x_offset, double y_offset) {
    GetGUI()->_camera->CameraZoom(y_offset);
}

GUI* GUI::theGUI = nullptr;

GUI *GUI::GetGUI() {
    if (theGUI == nullptr) {
        throw std::logic_error("There is no GUI inited");
    } else {
        return theGUI;
    }
}