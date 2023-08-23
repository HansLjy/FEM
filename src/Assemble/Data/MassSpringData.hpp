#pragma once

#include "Data.hpp"
#include "JsonUtil.h"

struct MassSpringData : public SampledObjectData {
	explicit MassSpringData(const json& config);

	/**
	 * @param topo *2D* connection info of the object
	 * @param density Surface density, mass will be evenly distributed
	 *                among vertices of the same triangle
	 * @param stiffness Stiffness of the spring
	 * @param IFN Initial face number: This is for consistency with normal mass-spring
	 *            system. Basicly, IFN is the number of faces simulated at the beginning
	 *            of the simulation. If IFN < 0(which is the default setting), all faces
	 *            are simulated, falling back to the normal case.
	 * @note If dynamic modification of the data is required, the vertices and faces must
	 *       satisfy the following order: the nth face should be connected to at least one
	 *       of the faces in [0, n) by edge and the vertices follow the order of adding.
	 */
	MassSpringData(const VectorXd& x_rest, const MatrixXi& topo, double density, double stiffness, int IFN = -1);

	void AddTriangle(int id1, int id2, const Vector3d& position);
	void AddTriangle(int id1, int id2, int id3);

	double _density;	// surface density
	double _stiffness;	// stiffness matrix
	VectorXd _x_rest;
	VectorXd _rest_length;
};