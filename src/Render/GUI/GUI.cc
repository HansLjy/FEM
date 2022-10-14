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

GLFWwindow* window;
int GUI::width = 800;
int GUI::height = 600;
std::string GUI::title = "Mesh Viewer";

int GUI::major_version = 3;
int GUI::minor_version = 3;

std::string GUI::vs_path = SHADER_PATH "/VertexShader.vert";
std::string GUI::fs_path = SHADER_PATH "/FragmentShader.frag";


Camera GUI::camera(
    glm::vec3(0.0f, 0.0f, 5.0f),
    glm::vec3(0.0f, 0.0f, -1.0f),
    glm::vec3(0.0f, 1.0f, 0.0f)
);

Light GUI::light(
    glm::vec3(0.0f, 10.0f, 0.0f),
    glm::vec3(0.2f, 0.2f, 0.2f),
    glm::vec3(0.5f, 0.5f, 0.5f),
    glm::vec3(1.0f, 1.0f, 1.0f),
    1.0f, 0.35f, 0.44f
);

void GUI::MainLoop() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, major_version);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, minor_version);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (window == NULL) {
        spdlog::error("Failure in creating window");
        glfwTerminate();
        exit(-1);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, GUI::ResizeCB);
    glfwSetScrollCallback(window, GUI::ScrollCB);
    glfwSetCursorPosCallback(window, GUI::MouseMovementCB);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
        glfwTerminate();
        exit(-1);
    }

    Shader shader_program(vs_path, fs_path);

    float last_frame = static_cast<float>(glfwGetTime());
    float cur_frame;

    Material material(
        glm::vec3(1.0f, 0.5f, 0.31f),
        glm::vec3(1.0f, 0.5f, 0.31f),
        glm::vec3(0.5f, 0.5f, 0.5f),
        32.0f
    );

    Scene scene;
    InitializeScene(scene);

    while (!glfwWindowShouldClose(window)) {
        cur_frame = static_cast<float>(glfwGetTime());
        ProcessKeyboard(cur_frame - last_frame);
        last_frame = cur_frame;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        shader_program.Use();

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 projection = camera.GetProjectionMatrix((float)width / (float)height);

        shader_program.SetFloat("model", model);
        shader_program.SetFloat("view", view);
        shader_program.SetFloat("projection", projection);

        shader_program.SetFloat("eye", camera._eye);

        shader_program.SetFloat("light.source", camera._eye);       // the light always comes from your eyes
        shader_program.SetFloat("light.ambient", light._ambient);
        shader_program.SetFloat("light.diffuse", light._diffuse);
        shader_program.SetFloat("light.specular", light._specular);

        shader_program.SetFloat("material.ambient", material._ambient);
        shader_program.SetFloat("material.diffuse", material._diffuse);
        shader_program.SetFloat("material.specular", material._specular);
        shader_program.SetFloat("material.shininess", material._shininess);

        Processing(scene);

        scene.Draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
}



void GUI::ResizeCB(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

#define PRESS(key) glfwGetKey(window, GLFW_KEY_##key) == GLFW_PRESS
void GUI::ProcessKeyboard(float duration) {
    if (PRESS(W)) {
        camera.CameraMovement(CameraMovementType::kForward, duration);
    }
    if (PRESS(S)) {
        camera.CameraMovement(CameraMovementType::kBackward, duration);
    }
    if (PRESS(A)) {
        camera.CameraMovement(CameraMovementType::kLeftward, duration);
    }
    if (PRESS(D)) {
        camera.CameraMovement(CameraMovementType::kRightward, duration);
    }
    if (PRESS(Q)) {
        camera.CameraMovement(CameraMovementType::kUpward, duration);
    }
    if (PRESS(E)) {
        camera.CameraMovement(CameraMovementType::kDownward, duration);
    }
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
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

    camera.CameraRotation(x_pos - last_x, last_y - y_pos);
    last_x = x_pos;
    last_y = y_pos;
}

void GUI::ScrollCB(GLFWwindow *window, double x_offset, double y_offset) {
    camera.CameraZoom(y_offset);
}