#include "Camera.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include <cassert>
#include <math.h>
#include "spdlog/spdlog.h"

Camera::Camera(const glm::vec3& eye, const glm::vec3& front, const glm::vec3& up)
    : _eye(eye),
      _front(glm::normalize(front)),
      _up(glm::normalize(up - glm::dot(up, front) * front)) {}

glm::mat4 Camera::GetViewMatrix() {
    return glm::lookAt(_eye, _eye + _front, _up);
}

glm::mat4 Camera::GetProjectionMatrix(float aspect) {
    return glm::perspective(_fov, aspect, kZNear, kZFar);
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
            _eye += kMovementSpeed * delta_time * glm::normalize(glm::cross(_up, _front));
            break;
        case CameraMovementType::kRightward:
            _eye += kMovementSpeed * delta_time * glm::normalize(glm::cross(_front, _up));
            break;
        case CameraMovementType::kUpward:
            _eye += kMovementSpeed * delta_time * _up;
            break;
        case CameraMovementType::kDownward:
            _eye -= kMovementSpeed * delta_time * _up;
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

void Camera::CameraRotation(float delta_x, float delta_y) {
    _yaw += kMouseSensitivity * delta_x;
    _pitch += kMouseSensitivity * delta_y;

    if (_pitch > 89.0f) {
        _pitch = 89.0f;
    } else if (_pitch < -89.0f) {
        _pitch = -89.0f;
    }

    _front.x = cos(glm::radians(_pitch)) * cos(glm::radians(_yaw));
    _front.z = cos(glm::radians(_pitch)) * sin(glm::radians(_yaw));
    _front.y = sin(glm::radians(_pitch));
}