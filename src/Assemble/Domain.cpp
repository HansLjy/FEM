//
// Created by hansljy on 11/9/22.
//

#include "Domain.h"

#include "JsonUtil.h"

Domain::Domain(const nlohmann::json &config)
    : System(config["system"]) {

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
    delete _target;
    _target = new DomainTarget(*this);
}

Domain::~Domain() {
    for (const auto& subdomain : _subdomains) {
        delete subdomain;
    }
}

void DomainTarget::GetExternalForce(Ref<VectorXd> force) const {
    int current_row = 0;
    for (const auto& obj : _domain->_objs) {
        force.segment(current_row, obj->GetDOF()) = obj->GetExternalForce(_domain->_frame_rotation, _domain->_frame_x);
        current_row += obj->GetDOF();
    }
    force += _domain->_interface_force + _domain->_inertial_force;
}

double Domain::GetTotalMass() const {
    double mass = 0;
    for (const auto& obj : _objs) {
        mass += obj->GetTotalMass();
    }
    return mass;
}

Vector3d Domain::GetTotalExternalForce() const {
    Vector3d total_external_force = Vector3d::Zero();
    for (const auto& obj : _objs) {
        total_external_force += obj->GetTotalExternalForce(_frame_rotation, _frame_x);
    }
    return total_external_force;
}

void Domain::CalculateTotalMass() {
    _total_mass = GetTotalMass();
    for (const auto& subdomain : _subdomains) {
        subdomain->CalculateTotalMass();
        _total_mass += subdomain->_total_mass;
    }
}

void Domain::CalculateTotalExternalForce() {
    _total_external_force = GetTotalExternalForce();
    for (const auto& subdomain : _subdomains) {
        subdomain->CalculateTotalExternalForce();
        _total_external_force += _frame_rotation.transpose() * subdomain->_frame_rotation * subdomain->_total_external_force;
    }

}

void Domain::CalculateInterfaceForce() {
    if(_interface_force.size() != _dof) {
        _interface_force.resize(_dof);
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
    if(_inertial_force.size() != _dof) {
        _inertial_force.resize(_dof);
    }
    int current_row = 0;
    for (const auto& obj : _objs) {
        _inertial_force.segment(current_row, obj->GetDOF()) = obj->GetInertialForce(
            _frame_v, _frame_a, _frame_angular_velocity,
            _frame_angular_acceleration, _frame_rotation
        );
        current_row += obj->GetDOF();
    }
}

#include "JsonUtil.h"

void Domain::AddSubdomain(Domain &subdomain, const nlohmann::json &position) {
    _subdomains.push_back(&subdomain);
    _subdomain_rest_rotations.push_back(Matrix3d(AngleAxisd(double(position["angle"]) / 180.0 * EIGEN_PI, Json2Vec(position["axis"]))));
}

void Domain::CalculateLumpedMass() {
    if (_lumped_mass.rows() != _dof || _lumped_mass.cols() != _dof) {
        _lumped_mass.resize(_dof, _dof);
        // TODO update settings to avoid this.
    }
    int num_subdomains = _subdomains.size();
    _lumped_mass.setZero();
    for (int i = 0; i < num_subdomains; i++) {
        _lumped_mass += _subdomains[i]->_total_mass * _subdomain_projections[i].transpose() * _subdomain_projections[i];
    }
}

void DomainTarget::GetMass(COO &coo, int offset_x, int offset_y) const {
    const auto& domain = dynamic_cast<const Domain*>(_system);
    SystemTarget::GetMass(coo, offset_x, offset_y);
    for (int i = 0; i < domain->_lumped_mass.outerSize(); i++) {
        for (SparseMatrixXd::InnerIterator it(domain->_lumped_mass, i); it; ++it) {
            coo.push_back(Tripletd(it.row() + offset_x, it.row() + offset_y, it.value()));
        }
    }
}

void Domain::UpdateSettings(const json &config) {
    System::UpdateSettings(config);
    const int num_subdomains = _subdomains.size();
    const auto& subdomains_config = config["subdomains"];
    for (int i = 0; i < num_subdomains; i++) {
        const auto& position = subdomains_config[i]["position"];
        _subdomain_projections.push_back(GetSubdomainProjection(position));
        RecordSubdomain(position);
    }
    if (num_subdomains) {
        VectorXd a(_dof);
        a.setZero();
        CalculateSubdomainFrame(a);
    }
    for (int i = 0; i < num_subdomains; i++) {
        _subdomains[i]->UpdateSettings(subdomains_config[i]);
    }
}

std::unique_ptr<ObjectIterator> Domain::GetIterator() {
    return std::unique_ptr<ObjectIterator>(new DomainIterator(*this));
}

void Domain::BottomUpCalculation() {
    CalculateTotalMass();
    CalculateTotalExternalForce();
}

void Domain::TopDownCalculationPrev() {
    CalculateInterfaceForce();
    CalculateInertialForce();
    CalculateLumpedMass();
}

DomainIterator::DomainIterator(Domain& domain)
    : ObjectIterator(domain._subdomains.empty() && domain._objs.empty()) {
    _current = &domain;
    _cur_obj_id = 0;
    _cur_size = _current->_objs.size();
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
            _cur_size = _current->_objs.size();
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
            _cur_size = _current->_objs.size();
        }
    }
}

Object *DomainIterator::GetObject() {
    return _current->_objs[_cur_obj_id];
}

Matrix3d DomainIterator::GetRotation() {
    return _current->_frame_rotation;
}

Vector3d DomainIterator::GetTranslation() {
    return _current->_frame_x;
}

#include "Domain/TreeDomain.h"

BEGIN_DEFINE_XXX_FACTORY(Domain)
    ADD_PRODUCT("tree-domain", TreeDomain)
END_DEFINE_XXX_FACTORY