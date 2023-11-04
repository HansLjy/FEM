#pragma once
#include "EigenAll.h"

namespace LeiLan {
	double TriangleArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);

	void TriangleNormal(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, Vector3d& n);

	bool DistanceToLine(const Vector3d & p, const Vector3d & v0, const Vector3d & v1, double& dist, double & t, Vector3d & fp);

	bool SameSize(const Vector3d& p1, const Vector3d& p2, const Vector3d& a, const Vector3d& b);

	bool InsideTriangle(const Vector3d& p, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);


	void TriBarycentericCoordinate(const Vector3d& p, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, Vector3d& coordinate);

	bool PointinTriangle1(Vector3d A, Vector3d B, Vector3d C, Vector3d P);

	bool SameSide(Vector3d A, Vector3d B, Vector3d C, Vector3d P);


	double VertexEdgeSqDistance(Vector3d v, Vector3d ea, Vector3d eb, double& r, Vector3d& dir);

	double EdgeEdgeSqDistance(Vector3d xi, Vector3d xj, Vector3d xa, Vector3d xb, double& r, double& s, Vector3d& dir);

	double VertexTriangleDistance(Vector3d xi, Vector3d xa, Vector3d xb, Vector3d xc, double& ba, double& bb, double& bc, Vector3d& dir);
}

