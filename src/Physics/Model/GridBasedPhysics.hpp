#include "Physics.hpp"

template<class ProxyModel>
class GridBasedPhysics : public ProxyModel {
public:
	explicit GridBasedPhysics(const json& config) : ProxyModel(config) {}
	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);
};

template<class ProxyModel>
template<class Data> void GridBasedPhysics<ProxyModel>::SetCoordinate(Data *obj, const Ref<const VectorXd> &x) {
	obj->_x.head(obj->_dof) = x;
	obj->UpdateProxyPosition();
}
