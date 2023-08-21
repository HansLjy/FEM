#pragma once
#include "Data.hpp"
#include "SpringMassData.hpp"

struct GridMeshData : public ReducedObjectData<SpringMassData> {

	/**
	 * @param mat OUTPUT, an 3 x N matrix containing the coordinates
	 *            of vertices following the order they were sent in
	 * @param x   INPUT, generalized coordinates of this object
	 */
	void GetSurfacePrimitive(Ref<Matrix3Xd> mat, const VectorXd& x);
	void AddTriangle(int index1, int index2, const Vector3d& position);

private:
	// Voxelization of the current triangle mesh
	void Vox();
};