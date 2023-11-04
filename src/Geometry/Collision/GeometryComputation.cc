#include "GeometryComputation.h"

#define IS_QEAL_ZERO(a) (std::abs(a) < 1e-14)

double LeiLan::TriangleArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2)
{
	return 0.5 * fabs(((v1 - v0).cross(v2 - v0)).norm());
}

void LeiLan::TriangleNormal(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, Vector3d& n)
{
	Vector3d vec1 = v1 - v0;
	Vector3d vec2 = v2 - v0;
	n = vec1.cross(vec2);
	n.normalize();
}

bool LeiLan::DistanceToLine(const Vector3d & p, const Vector3d & v0, const Vector3d & v1, double& dist, double & t, Vector3d & fp)
{
	Vector3d v0v1 = v1 - v0;
	Vector3d pv0 = v0 - p;
	Vector3d pv1 = v1 - p;

	double area = abs((v0v1.cross(pv0)).norm());

	if (!IS_QEAL_ZERO(v0v1.norm()))
	{
		dist = area / v0v1.norm();
		t = (pv0.dot(pv0) - pv0.dot(pv1)) / (pv0.dot(pv0) + pv1.dot(pv1) - 2 * pv0.dot(pv1));
		fp = (1.0 - t) * v0 + t * v1;
		return true;
	}
	else return false;
}

bool LeiLan::SameSize(const Vector3d& p1, const Vector3d& p2, const Vector3d& a, const Vector3d& b)
{
	Vector3d cp1 = (b - a).cross(p1 - a);
	Vector3d cp2 = (b - a).cross(p2 - a);
	if (!IS_QEAL_ZERO(cp1.dot(cp2)))
		return true;
	else
		return false;
}

bool LeiLan::InsideTriangle(const Vector3d& p, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2)
{
	if (SameSize(p, v0, v1, v2) && SameSize(p, v1, v0, v2) && SameSize(p, v2, v1, v0))
		return true;
	return false;

}

void LeiLan::TriBarycentericCoordinate(const Vector3d& p, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, Vector3d& coordinate)
{
	double area = TriangleArea(v0, v1, v2);
	coordinate[0] = TriangleArea(p, v1, v2) / area;
	coordinate[1] = TriangleArea(p, v0, v2) / area;
	coordinate[2] = TriangleArea(p, v1, v0) / area;
}


bool LeiLan::SameSide(Vector3d A, Vector3d B, Vector3d C, Vector3d P)
{
	Vector3d AB = B - A;
	Vector3d AC = C - A;
	Vector3d AP = P - A;

	Vector3d v1 = AB.cross(AC);
	Vector3d v2 = AB.cross(AP);

	// v1 and v2 should point to the same direction
	return v1.dot(v2) >= 0;
}

// Same side method
// Determine whether point P in triangle ABC
bool LeiLan::PointinTriangle1(Vector3d A, Vector3d B, Vector3d C, Vector3d P)
{
	return SameSide(A, B, C, P) &&
		SameSide(B, C, A, P) &&
		SameSide(C, A, B, P);
}


double LeiLan::VertexEdgeSqDistance(Vector3d v, Vector3d ea, Vector3d eb, double& r, Vector3d& dir)
{
	Vector3d va, ba;
	va = v - ea;
	ba = eb - ea;

	double va_ba = va.dot(ba);
	double ba_ba = ba.dot(ba);

	if (va_ba < 0)
		r = 0;
	else if (va_ba > ba_ba)
		r = 1;
	else r = va_ba / ba_ba;

	dir = v - ((1.0 - r) * ea + r * eb);

	return dir.dot(dir);
}

double LeiLan::EdgeEdgeSqDistance(Vector3d xi, Vector3d xj, Vector3d xa, Vector3d xb, double& r, double& s, Vector3d& dir)
{
	Vector3d xba, xji, xai;
	xba = xb - xa;
	xji = xj - xi;
	xai = xa - xi;
	dir = xji.cross(xba);
	double nn = dir.dot(dir);
	Vector3d temp;
	temp = xai.cross(xji);

	double weight_aiji = dir.dot(temp);

	temp = xai.cross(xba);
	double weight_aiba = dir.dot(temp);

	if (nn > 1e-24f && weight_aiji >= 0 && weight_aiji <= nn && weight_aiba >= 0 && weight_aiba <= nn)
	{
		r = weight_aiba / nn;
		s = weight_aiji / nn;
	}
	else
	{
		double minDistance = 999999999;
		double distance, v;
		Vector3d Ndir;
		if (weight_aiba < 0 && ((distance = VertexEdgeSqDistance(xi, xa, xb, v, Ndir)) < minDistance))
		{
			minDistance = distance;
			r = 0;
			s = v;
		}
		if (weight_aiba > nn && ((distance = VertexEdgeSqDistance(xj, xa, xb, v, Ndir)) < minDistance))
		{
			minDistance = distance;
			r = 1;
			s = v;
		}
		if (weight_aiji < 0 && ((distance = VertexEdgeSqDistance(xa, xi, xj, v, Ndir)) < minDistance))
		{
			minDistance = distance;
			r = v;
			s = 0;
		}
		if (weight_aiji > nn && ((distance = VertexEdgeSqDistance(xb, xi, xj, v, Ndir)) < minDistance))
		{
			minDistance = distance;
			r = v;
			s = 1;
		}
	}
	dir = xi * (1.0 - r) + xj * r - xa * (1.0 - s) - xb * s;
	return dir.dot(dir);
}

double LeiLan::VertexTriangleDistance(Vector3d xi, Vector3d xa, Vector3d xb, Vector3d xc, double& ba, double& bb, double& bc, Vector3d& dir)
{
	Vector3d xba, xca, xia;
	xba = xb - xa;
	xca = xc - xa;
	xia = xi - xa;
	dir = xba.cross(xca);

	double nn = dir.dot(dir);

	Vector3d temp;
	temp = xia.cross(xca);
	double weight_iaca = dir.dot(temp);
	temp = xba.cross(xia);
	double weight_baia = dir.dot(temp);

	if (nn > 1e-24f && weight_iaca >= 0 && weight_baia >= 0 && nn - weight_iaca - weight_baia >= 0)
	{
		bb = weight_iaca / nn;
		bc = weight_baia / nn;
		ba = 1 - bb - bc;
	}
	else
	{
		double minDistance = 999999999;
		double r, distance;
		Vector3d N;
		if (nn - weight_iaca - weight_baia < 0 && ((distance = VertexEdgeSqDistance(xi, xb, xc, r, N)) < minDistance))
		{
			minDistance = distance;
			bb = 1 - r;
			bc = r;
			ba = 0;
		}
		if (weight_iaca < 0 && ((distance = VertexEdgeSqDistance(xi, xa, xc, r, N)) < minDistance))
		{
			minDistance = distance;
			bb = 0;
			bc = r;
			ba = 1 - bb - bc;
		}
		if (weight_baia < 0 && ((distance = VertexEdgeSqDistance(xi, xa, xb, r, N)) < minDistance))
		{
			minDistance = distance;
			bb = r;
			bc = 0;
			ba = 1 - bb - bc;
		}
	}
	dir = xi - ba * xa - bb * xb - bc * xc;
	return dir.dot(dir);
}
