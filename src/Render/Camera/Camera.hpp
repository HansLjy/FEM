#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "glm/glm.hpp"

enum class CameraMovementType {
    kForward,
    kBackward,
    kLeftward,
    kRightward,
    kUpward,
    kDownward
};


struct Camera {
    Camera(const glm::vec3& eye, const glm::vec3& front, const glm::vec3& up);

    glm::mat4 GetViewMatrix();
    glm::mat4 GetProjectionMatrix(float aspect);

    void CameraMovement(const CameraMovementType& movement_type, float delta_time);
    void CameraZoom(float delta_scroll);
    void CameraRotation(float delta_x, float delta_y);

    glm::vec3 _eye;
    glm::vec3 _front;
    glm::vec3 _up;      // positive y-axis

private:
    const float kMovementSpeed = 3.0f;
    const float kMouseSensitivity = 0.1f;
    const float kZoomingSpeed = 1.0f;

    const float kZNear = 0.1f;
    const float kZFar = 100.0f;

    float _fov = 45.0f;
    float _yaw = -90.0f;
    float _pitch = 0;
};

#endif