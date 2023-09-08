#pragma once

#include "EigenAll.h"

struct TriangleData {
	TriangleData(const Vector9d& x, const Vector9d& x_rest, double density, double stiffness, double ret_stiffness)
		: _x(x), _x_rest(x_rest), _density(density), _stiffness(stiffness), _ret_stiffness(ret_stiffness) {}

	const int _dof = 9;
	const int _num_points = 1;
	const int _num_edges = 3;
	const int _num_faces = 1;

	const Matrix<int, 3, 2> _edge_topo = (Matrix<int, 3, 2>()
		<< 0, 1,
		   1, 2,
		   2, 0
	).finished();
	const Matrix<int, 1, 3> _face_topo = (Matrix<int, 1, 3>() << 0, 1, 2).finished();
	const Matrix<int, 0, 4> _tet_topo;

	Vector9d _x;
	Vector9d _v = Vector9d::Zero();
	Vector9d _x_rest;
	double _density;
	double _stiffness;
	double _ret_stiffness;

	Vector3d _mass;
	double _total_mass;

};