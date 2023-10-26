#include "PDAssembler.hpp"

template<>
Caster<PDObject>* Caster<PDObject>::_the_factory = nullptr;

double PDAssembler::GetEnergy(const std::vector<PDObject>& objs, const Ref<const VectorXd> &x) const {
    double energy = 0;
    int offset = 0;
    for (const auto& obj : objs) {
        energy += obj.GetEnergy(x.segment(offset, obj.GetDOF()));
        offset += obj.GetDOF();
    }
    return energy;
}

SparseMatrixXd PDAssembler::GetGlobalMatrix(const std::vector<PDObject>& objs, int total_dof) const {
    COO coo;
    GetGlobalMatrix(objs, coo, 0, 0);
	SparseMatrixXd global_matrix(total_dof, total_dof);
    global_matrix.setFromTriplets(coo.begin(), coo.end());
	return global_matrix;
}

void PDAssembler::GetGlobalMatrix(const std::vector<PDObject>& objs, COO &coo, int x_offset, int y_offset) const {
    int cur_offset = 0;
    for (const auto& obj : objs) {
        obj.GetGlobalMatrix(coo, cur_offset + x_offset, cur_offset + y_offset);
        cur_offset += obj.GetDOF();
    }
}

void PDAssembler::LocalProject(
    const std::vector<PDObject>& objs,
    const Ref<const VectorXd> &x,
    Ref<VectorXd> y
) const {
    int cur_offset = 0;
    for (const auto& obj : objs) {
        obj.LocalProject(x.segment(cur_offset, obj.GetDOF()), y.segment(cur_offset, obj.GetDOF()));
        cur_offset += obj.GetDOF();
    }
}


