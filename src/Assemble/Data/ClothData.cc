#include "ClothData.hpp"
#include "unsupported/Eigen/KroneckerProduct"
#include "ExternalForce/ExternalForce.hpp"
#include "InitUtils.hpp"

template<>
Factory<ExternalForce<ClothData>>* Factory<ExternalForce<ClothData>>::_the_factory = nullptr;

ClothData::ClothData(
    const VectorXd& x,
    const MatrixXi& topo,
    const VectorXd& mass,
    double density,
    double k_stretch, double k_shear,
    double k_bend_max, double k_bend_min,
    const Vector2d& max_bend_dir,
    const MatrixXd& uv_coord,
    double stretch_u, double stretch_v) :
    SampledObjectData(x, mass, 2, topo),
    _k_stretch(k_stretch), _k_shear(k_shear),
    _stretch_u(stretch_u), _stretch_v(stretch_v),
    _uv_coord(uv_coord),
    _area(InitializationUtils::Surface::CalculateArea(x, topo)),
    _inv(InitializationUtils::Surface::CalculateInv(uv_coord, topo)),
    _pFpx(InitializationUtils::Surface::CalculatePFPX(uv_coord, topo)){
    
    _internal_edge = TopoUtil::GetInternalEdge(topo);
    _num_internal_edges = _internal_edge.rows();

    _internal_edge_length.resize(_num_internal_edges);
    _internal_edge_k_bend.resize(_num_internal_edges);

    for (int i = 0; i < _num_internal_edges; i++) {
        const RowVector3i indices = _internal_edge.row(i);
        _internal_edge_length(i) = (
            _x.segment<3>(3 * indices[0]) -
            _x.segment<3>(3 * indices[1])
        ).norm();

        const Vector2d uv_direction = _uv_coord.row(indices[1]) - _uv_coord.row(indices[0]);
        const double delta_u = uv_direction.dot(max_bend_dir);
        const double delta_v = - uv_direction(0) * max_bend_dir(1) + uv_direction(1) * max_bend_dir(0);
        _internal_edge_k_bend(_num_internal_edges) =
                (k_bend_max * delta_u * delta_u + k_bend_min * delta_v * delta_v) /
                (delta_u * delta_u + delta_v * delta_v);
 
    }
}