//
// Created by hansljy on 11/9/22.
//

#include "Domain.h"

#include "JsonUtil.h"

Domain::Domain(const nlohmann::json &config)
    : _system(config["system"]) {

    if (config["is-root"]) {
        _frame_x = Json2Vec(config["x"]);
        _frame_v = Vector3d::Zero();
        _frame_a = Vector3d::Zero();
        _frame_rotation = Json2Matrix3d(config["rotation"]);
        _frame_angular_velocity = Vector3d::Zero();
        _frame_angular_acceleration = Vector3d::Zero();
    }
    const auto& subdomains_config = config["subdomains"];
    for (const auto& subdomain_config : subdomains_config) {
        Domain* subdomain = DomainFactory::GetDomain(subdomain_config["type"], subdomain_config);
        AddSubdomain(*subdomain, subdomain_config["position"]);
    }
}

void Domain::SetFrame(const Eigen::Vector3d &x, const Eigen::Vector3d &v, const Eigen::Vector3d &a,
                      const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &angular_acceleration,
                      const Eigen::Matrix3d &rotation) {
    _frame_x = x;
    _frame_v = v;
    _frame_a = a;
    _frame_angular_velocity = angular_velocity;
    _frame_angular_acceleration = angular_acceleration;
    _frame_rotation = rotation;
}

Domain::~Domain() {
    for (const auto& subdomain : _subdomains) {
        delete subdomain;
    }
}

Domain::Domain(const Domain &rhs)
    : _system(rhs._system),
      _subdomain_projections(rhs._subdomain_projections),
      _subdomain_rest_rotations(rhs._subdomain_rest_rotations),
      _frame_x(rhs._frame_x), _frame_v(rhs._frame_v), _frame_a(rhs._frame_a),
      _frame_angular_velocity(rhs._frame_angular_velocity),
      _frame_angular_acceleration(rhs._frame_angular_acceleration),
      _frame_rotation(rhs._frame_rotation),
      _total_mass(rhs._total_mass),
      _total_external_force(rhs._total_external_force),
      _interface_force(rhs._interface_force),
      _inertial_force(rhs._inertial_force),
      _lumped_mass(rhs._lumped_mass) {
    for (const auto& subdomain : rhs._subdomains) {
        _subdomains.push_back(subdomain->Clone());
    }
    _total_mass = rhs._total_mass;
}


void Domain::CalculateTotalMass() {
    _total_mass = 0;
    for (const auto& subdomain : _subdomains) {
        subdomain->CalculateTotalMass();
        _total_mass += subdomain->_total_mass;
    }
    _total_mass += _system.GetTotalMass();
}

void Domain::CalculateTotalExternalForce() {
    _total_external_force = _system.GetTotalExternalForce(_frame_rotation, _frame_x);
    for (const auto& subdomain : _subdomains) {
        subdomain->CalculateTotalExternalForce();
        _total_external_force += _frame_rotation.transpose() * subdomain->_frame_rotation * subdomain->_total_external_force;
    }
}

void Domain::Preparation() {
    CalculateInterfaceForce();
    CalculateInertialForce();
    CalculateLumpedMass();
}

void Domain::CalculateInterfaceForce() {
    if(_interface_force.size() != _system._DOF) {
        _interface_force.resize(_system._DOF);
    }
    _interface_force.setZero();
    const int num_subdomains = _subdomains.size();
    Matrix3d frame_angular_velocity = HatMatrix(_frame_angular_velocity);
    Matrix3d omega = _frame_rotation.transpose() * frame_angular_velocity;
    Matrix3d omega2 = omega * frame_angular_velocity;
    Matrix3d alpha = _frame_rotation.transpose() * HatMatrix(_frame_angular_acceleration);
    for (int i = 0; i < num_subdomains; i++) {
        Vector3d v_rel = _subdomains[i]->_frame_v - _frame_v;
        Vector3d x_rel = _subdomains[i]->_frame_x - _frame_x;
        _interface_force += _subdomain_projections[i].transpose()
                          * _frame_rotation.transpose() * _subdomains[i]->_frame_rotation
                          * _subdomains[i]->_total_external_force;
        _interface_force -= _subdomain_projections[i].transpose()
                          * _subdomains[i]->_total_mass * (_frame_a + 2 * omega * v_rel + alpha * x_rel + omega2 * x_rel);
    }
}

void Domain::CalculateInertialForce() {
    if(_inertial_force.size() != _system._DOF) {
        _inertial_force.resize(_system._DOF);
    }
    int current_row = 0;
    for (const auto& obj : _system._objs) {
        _inertial_force.segment(current_row, obj->GetDOF()) = obj->GetInertialForce(
            _frame_v, _frame_a, _frame_angular_velocity,
            _frame_angular_acceleration, _frame_rotation
        );
        current_row += obj->GetDOF();
    }
}

#include "JsonUtil.h"

void Domain::AddSubdomain(const Domain &subdomain, const nlohmann::json &position) {
    _subdomains.push_back(subdomain.Clone());
    _subdomain_rest_rotations.push_back(Matrix3d(AngleAxisd(double(position["angle"]) / 180.0 * EIGEN_PI, Json2Vec(position["axis"]))));
}

void Domain::CalculateLumpedMass() {
    const int dof = _system._DOF;
    if (_lumped_mass.rows() != dof || _lumped_mass.cols() != dof) {
        _lumped_mass.resize(dof, dof);
        // TODO update settings to avoid this.
    }
    int num_subdomains = _subdomains.size();
    _lumped_mass.setZero();
    for (int i = 0; i < num_subdomains; i++) {
        _lumped_mass += _subdomains[i]->_total_mass * _subdomain_projections[i].transpose() * _subdomain_projections[i];
    }
}

void Domain::GetMass(SparseMatrixXd &mass) const {
    _system.GetMass(mass);
    mass += _lumped_mass;
}

double Domain::GetEnergy() const {
    VectorXd x = _system.GetCoordinate();
    return _system.GetEnergy(_frame_rotation, _frame_x) - x.dot(_interface_force + _inertial_force);
}

double Domain::GetEnergy(const Eigen::VectorXd &x) const {
    return _system.GetEnergy(x, _frame_rotation, _frame_x) - x.dot(_interface_force + _inertial_force);
}

VectorXd Domain::GetEnergyGradient() const {
    return _system.GetEnergyGradient(_frame_rotation, _frame_x) - _interface_force - _inertial_force;
}

void Domain::UpdateSettings(const json &config) {
    _system.UpdateSettings(config);
    const int num_subdomains = _subdomains.size();
    const auto& subdomains_config = config["subdomains"];
    for (int i = 0; i < num_subdomains; i++) {
        const auto& position = subdomains_config[i]["position"];
        _subdomain_projections.push_back(GetSubdomainProjection(position));
        RecordSubdomain(position);
        _subdomains[i]->UpdateSettings(subdomains_config[i]);
    }
    if (!_subdomains.empty()) {
        CalculateSubdomainFrame(VectorXd(_system._DOF));
    }
}

ObjectIterator *Domain::GetIterator() {
    return new DomainIterator(*this);
}

DomainIterator::DomainIterator(Domain& domain)
    : ObjectIterator(domain._subdomains.empty() && domain._system._objs.empty()) {
    _current = &domain;
    _cur_obj_id = 0;
    _cur_size = _current->_system._objs.size();
}

void DomainIterator::Forward() {
    _cur_obj_id++;
    if (_cur_obj_id >= _cur_size) {
        if (!_current->_subdomains.empty()) {
            // dig deeper
            _ancestors.push(_current);
            _subdomain_ids.push(0);
            _current = _current->_subdomains[0];
            _cur_obj_id = 0;
            _cur_size = _current->_system._objs.size();
        } else {
            // go back
            if (_ancestors.empty()) {
                _is_done = true;
                return;
            }
            Domain *parent = _ancestors.top();
            int domain_id = _subdomain_ids.top() + 1;

            while (domain_id >= parent->_subdomains.size()) {
                _ancestors.pop();
                _subdomain_ids.pop();
                if (_ancestors.empty()) {
                    _is_done = true;
                    return;
                }
                parent = _ancestors.top();
                domain_id = _subdomain_ids.top() + 1;
            }
            _ancestors.top() = parent;
            _subdomain_ids.top() = domain_id;
            _current = parent->_subdomains[domain_id];
            _cur_obj_id = 0;
            _cur_size = _current->_system._objs.size();
        }
    }
}

Object *DomainIterator::GetObject() {
    return _current->_system._objs[_cur_obj_id];
}

Matrix3d DomainIterator::GetRotation() {
    return _current->_frame_rotation;
}

Vector3d DomainIterator::GetTranslation() {
    return _current->_frame_x;
}

#include "Tree/TreeDomain.h"

BEGIN_DEFINE_XXX_FACTORY(Domain)
    ADD_PRODUCT("tree-domain", TreeDomain)
END_DEFINE_XXX_FACTORY