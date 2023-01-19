#pragma once
#include "EigenAll.h"

MatrixXi GetFaceTopo(const MatrixXi& tet_topo);
MatrixXi GetEdgeTopo(const MatrixXi& face_topo);

void GenerateSurfaceTopo3D(const MatrixXi& tet_topo, MatrixXi& face_topo, MatrixXi& edge_topo);
void GenerateSurfaceTopo2D(const MatrixXi& face_topo, MatrixXi& edge_topo);