#include "PDAssembler.hpp"

template<>
Caster<PDObject>* Caster<PDObject>::_the_factory = nullptr;

void PDAssembler::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    InterfaceContainer::BindObjects(begin, end);
    _total_dof = 0;
    for (const auto& obj : _objs) {
        _total_dof += obj.GetDOF();
    }
}

void PDAssembler::GetGlobalMatrix(SparseMatrixXd& mass) const {
    COO coo;
    GetGlobalMatrix(coo, 0, 0);
    mass.resize(_total_dof, _total_dof);
    mass.setFromTriplets(coo.begin(), coo.end());
}

void PDAssembler::GetGlobalMatrix(COO &coo, int x_offset, int y_offset) const {
    int cur_offset = 0;
    for (const auto& obj : _objs) {
        obj.GetGlobalMatrix(coo, cur_offset + x_offset, cur_offset + y_offset);
        cur_offset += obj.GetDOF();
    }
}

void PDAssembler::LocalProject(const Ref<const VectorXd> &x, Ref<VectorXd> y, int offset) const {
    int cur_offset = offset;
    for (const auto& obj : _objs) {
        obj.LocalProject(x.segment(cur_offset, obj.GetDOF()), y.segment(cur_offset, obj.GetDOF()));
        cur_offset += obj.GetDOF();
    }
}


