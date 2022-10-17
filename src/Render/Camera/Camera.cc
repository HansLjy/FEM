#include "Camera.hpp"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include <cassert>
#include <cmath>
#include "spdlog/spdlog.h"

Camera::Camera(const glm::vec3 &eye, const glm::vec3 &front, const glm::vec3 &world_up,
               float near, float far)
    : _eye(eye),
      _front(glm::normalize(front)),
      _right(glm::normalize(glm::cross(front, world_up))),
      _up(glm::normalize(glm::cross(_right, front))),
      _near(near),
      _far(far),
      _world_up(glm::normalize(world_up)),
      _world_front(glm::normalize(glm::cross(world_up, _right))),
      _world_right(_right),
      _yaw(0) {
    double cos_pitch = glm::dot(_world_front, _front);
    double sin_pitch = glm::dot(glm::cross(_world_front, _front), _right);
    _pitch = atan2(sin_pitch, cos_pitch) / glm::pi<float>() * 180.0f;
}

glm::mat4 Camera::GetViewMatrix() {
    return glm::lookAt(_eye, _eye + _front, _world_up);
}

glm::mat4 Camera::GetProjectionMatrix(float aspect) {
    return glm::perspective(_fov, aspect, _near, _far);
}

void Camera::CameraMovement(const CameraMovementType &movement_type, float delta_time) {
    switch (movement_type) {
        case CameraMovementType::kBackward:
            _eye -= kMovementSpeed * delta_time * _front;
            break;
        case CameraMovementType::kForward:
            _eye += kMovementSpeed * delta_time * _front;
            break;
        case CameraMovementType::kLeftward:
            _eye -= kMovementSpeed * delta_time * _right;
            break;
        case CameraMovementType::kRightward:
            _eye += kMovementSpeed * delta_time * _right;
            break;
        case CameraMovementType::kUpward:
            _eye += kMovementSpeed * delta_time * _world_up;
            break;
        case CameraMovementType::kDownward:
            _eye -= kMovementSpeed * delta_time * _world_up;
            break;
    }
}

void Camera::CameraZoom(float delta_scroll) {
    _fov += kZoomingSpeed * delta_scroll;
    if (_fov > 45.0f) {
        _fov = 45.0f;
    } else if (_fov < 1.0f) {
        _fov = 1.0f;
    }
}

#include <iostream>

void Camera::CameraRotation(float delta_x, float delta_y) {
    _yaw += kMouseSensitivity * delta_x;
    _pitch += kMouseSensitivity * delta_y;

    if (_pitch > 89.0f) {
        _pitch = 89.0f;
    } else if (_pitch < -89.0f) {
        _pitch = -89.0f;
    }

    _front = float(cos(glm::radians(_pitch)) * sin(glm::radians(_yaw))) * _world_right
           + float(cos(glm::radians(_pitch)) * cos(glm::radians(_yaw))) * _world_front
           + float(sin(glm::radians(_pitch))) * _world_up;

    _right = glm::normalize(glm::cross(_front, _world_up));
    _up = glm::cross(_right, _front);
}