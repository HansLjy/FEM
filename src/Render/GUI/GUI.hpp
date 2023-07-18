#ifndef GUI_HPP
#define GUI_HPP

#include "Camera/Camera.hpp"
#include "Light/Light.hpp"
#include "Scene/Scene.hpp"
#include "Shader/Shader.hpp"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include <string>

class GUI {
public:
    explicit GUI(const std::string& title = "Renderer", int width = 1000, int height = 750,
        int opengl_major_version = 3, int opengl_minor_version = 3);

    void SetCamera(const glm::vec3& eye, const glm::vec3& front, const glm::vec3& up);
    void SetLight(const glm::vec3 &position, const glm::vec3 &ambient, const glm::vec3 &diffuse, const glm::vec3 &specular,
                  float Kc, float Kl, float Kq);

    void MainLoop();
    virtual void Processing(Scene& scene) = 0;
    virtual void InitializeScene(Scene& scene) = 0;

    virtual ~GUI();
    GUI(const GUI& rhs) = delete;
    GUI& operator=(const GUI& rhs) = delete;

    static GUI* GetGUI();
    static void ResizeCB(GLFWwindow* window, int width, int height);
    static void MouseMovementCB(GLFWwindow* window, double x_pos, double y_pos);
    static void ScrollCB(GLFWwindow* window, double x_offset, double y_offset);

    void ProcessKeyboard(float duration);

private:
    GLFWwindow* _window;
    int _width;
    int _height;
    std::string _title;
    int _major_version;
    int _minor_version;
    std::string _vs_path;
    std::string _fs_path;

    Camera* _camera = nullptr;
    Light* _light = nullptr;

    unsigned int _floor_VAO = 0;
    bool _stop = false;

    static GUI* theGUI;
};


#endif