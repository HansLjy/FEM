#ifndef GUI_HPP
#define GUI_HPP

#include "Camera/Camera.hpp"
#include "Light/Light.hpp"
#include "Scene/Scene.hpp"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include <string>

class GUI {
public:
    void MainLoop();
    virtual void Processing(Scene& scene) = 0;
    virtual void InitializeScene(Scene& scene) = 0;

    static void ResizeCB(GLFWwindow* window, int width, int height);
    static void MouseMovementCB(GLFWwindow* window, double x_pos, double y_pos);
    static void ScrollCB(GLFWwindow* window, double x_offset, double y_offset);
    static void ProcessKeyboard(float duration);

private:
    static GLFWwindow* _window;
    static int width;
    static int height;
    static std::string title;
    static int major_version;
    static int minor_version;
    static std::string vs_path;
    static std::string fs_path;

    static Camera camera;
    static Light light;
};


#endif