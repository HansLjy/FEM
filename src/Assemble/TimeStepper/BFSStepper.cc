//
// Created by hansljy on 11/29/22.
//

#include "BFSStepper.hpp"

// BFSStepper::BFSStepper(const nlohmann::json &config) : TimeStepper(config) {
//     const auto& integrator_config = config["integrator"];
//     _integrator = Factory<Integrator>::GetInstance()->GetProduct(integrator_config["type"], integrator_config);
// }

// void BFSStepper::Bind(System &system) {
//     TimeStepper::Bind(system);
// 	_level_targets.clear();
// 	_levels = system._level_bar.size() - 1;
// 	for (int i = 0; i < _levels; i++) {
// 		_level_targets.push_back(
//             TargetFactory::GetTarget(system._all_objs, system._level_bar[i], _system->_level_bar[i + 1], _target_config["type"], _target_config)
//         );
// 	}
// }

// void BFSStepper::Step(double h) const {
// 	for (int i = 0; i < _system->_level_bar[1]; i++) {
// 		if (_system->_all_objs[i]->IsDecomposed()) {
// 			auto root = dynamic_cast<DecomposedObject*>(_system->_all_objs[i]);
// 			root->Aggregate();
// 		}
// 	}
//     for (int i = 0; i < _levels; i++) {
//         // [_level_bar[i], _level_bar[i + 1])
//         const auto& target = _level_targets[i];
//         VectorXd v(target->GetDOF()), v_new(target->GetDOF());
//         target->GetVelocity(v);
//         _integrator->Step(*target, h);
//         target->GetVelocity(v_new);
//         VectorXd a = (v_new - v) / h;

//         int cur_row = 0;
//         for (int j = _system->_level_bar[i]; j < _system->_level_bar[i + 1]; j++) {
// 			auto obj = _system->_all_objs[j];
// 			if (_system->_all_objs[j]->IsDecomposed()) {
// 				auto decomposed_obj = dynamic_cast<DecomposedObject*>(obj);
// 				decomposed_obj->Distribute(a.segment(cur_row, decomposed_obj->GetDOF()));
// 			}
//             cur_row += obj->GetDOF();
//         }
//     }
// }

// BFSStepper::~BFSStepper() noexcept {
//     delete _integrator;
//     for (const auto& target : _level_targets) {
//         delete target;
//     }
// }

// ParallelBFSStepper::ParallelBFSStepper(const json& config) : TimeStepper(config) {
//     const auto& integrator_config = config["integrator"];
//     _integrator = IntegratorFactory::GetIntegrator(integrator_config["type"], integrator_config);
// }

// void ParallelBFSStepper::Bind(System &system) {
//     TimeStepper::Bind(system);
//     _levels = system._level_bar.size() - 1;
//     for (int i = 0; i < system._all_objs.size(); i++) {
//         _targets.push_back(TargetFactory::GetTarget(system._all_objs, i, i + 1, _target_config["type"], _target_config));
//     }
// }

// #include <omp.h>

// void ParallelBFSStepper::Step(double h) const {
//     for (int i = 0; i < _system->_level_bar[1]; i++) {
// 		if (_system->_all_objs[i]->IsDecomposed()) {
// 			auto root = dynamic_cast<DecomposedObject*>(_system->_all_objs[i]);
// 			root->Aggregate();
// 		}
// 	}

//     for (int i = 0; i < _levels; i++) {
//         // [_level_bar[i], _level_bar[i + 1])
//         int begin = _system->_level_bar[i];
//         int end = _system->_level_bar[i + 1];
//         #pragma omp parallel for
//         for (int j = begin; j < end; j++) {
//             const auto& target = _targets[j];
//             VectorXd v(target->GetDOF()), v_new(target->GetDOF());
//             target->GetVelocity(v);
//             _integrator->Step(*target, h);
//             target->GetVelocity(v_new);
//             VectorXd a = (v_new - v) / h;

//             auto obj = _system->_all_objs[j];
//             if (_system->_all_objs[j]->IsDecomposed()) {
//                 auto decomposed_obj = dynamic_cast<DecomposedObject*>(obj);
//                 decomposed_obj->Distribute(a);
//             }
//         }
//     }
// }