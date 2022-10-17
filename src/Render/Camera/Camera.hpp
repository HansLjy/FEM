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
    Camera(const glm::vec3 &eye, const glm::vec3 &front, const glm::vec3 &world_up,
           float near = 0.1f, float far = 100.0f);

    glm::mat4 GetViewMatrix();
    glm::mat4 GetProjectionMatrix(float aspect);

    void CameraMovement(const CameraMovementType& movement_type, float delta_time);
    void CameraZoom(float delta_scroll);
    void CameraRotation(float delta_x, float delta_y);

    glm::vec3 _eye;
    glm::vec3 _front;
    glm::vec3 _right;
    glm::vec3 _up;

    float _near;
    float _far;

private:
    const float kMovementSpeed = 3.0f;
    const float kMouseSensitivity = 0.1f;
    const float kZoomingSpeed = 1.0f;

    const glm::vec3 _world_up;
    const glm::vec3 _world_front;    // the initial front axis
    const glm::vec3 _world_right;    // the initial right axis

    float _fov = 45.0f;
    float _yaw;
    float _pitch;
};

#endif