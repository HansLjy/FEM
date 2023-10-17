#include "Data/PDClothData.hpp"

#include "Coordinate/Coordinate.hpp"
#include "Coordinate/CoordinateAdapter.hpp"

#include "MassModel/MassModel.hpp"
#include "MassModel/MassModelAdapter.hpp"

#include "Object.hpp"

#include "PDEnergyModel/PDClothEnergyModel.hpp"
#include "PDEnergyModel/PDEnergyAdapter.hpp"

#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"

#include "Render/RenderShape.hpp"
#include "Render/RenderShapeAdapter.hpp"
#include "Render/RenderInterface.hpp"

#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionShape/CollisionShapeAdapter.hpp"

#include "Collision/CollisionShape/ImplicitShape/ImplicitSphere.hpp"

#include "TimeStepper/PDIPC.hpp"

class PDIPCCloth :
	public PDClothData,
    public CoordinateAdapter<BasicCoordinate, PDIPCCloth>,
    public MassModelAdapter<SampledObjectMassModel, PDIPCCloth>,
    public PDEnergyModelAdapter<PDClothEnergyModel, PDIPCCloth>,
    public ExternalForceContainerAdapter<PDClothData, PDIPCCloth>,
    public RenderShapeAdapter<SampledRenderShape, PDIPCCloth>,
	public CollisionShapeAdapter<SampledCollisionShape, PDIPCCloth> {

public:
	PDIPCCloth(const json& config):
        PDClothData(PDClothData::CreateFromFile(config)),
        PDEnergyModelAdapter(PDClothEnergyModel::CreateFromConfig(config["energy"])),
        ExternalForceContainerAdapter(ExternalForceContainer<PDClothData>::CreateFromConfig(config["external-forces"])),
        RenderShapeAdapter(SampledRenderShape::CreateFromConfig(config["render"])),
		CollisionShapeAdapter(config["collision"]){
		PDEnergyModelAdapter::Initialize();
		CollisionShapeAdapter::Initialize();
    }
};

const bool sampled_gravity_registered = ExternalForceRegistration::RegisterExternalForce<SampledObjectGravity, PDClothData>("gravity");
const bool pdipc_cloth_registered = CreatorRegistration::RegisterForCreator<PDIPCCloth>("cloth");
const bool pdipc_cloth_caster_registered = ObjectRegistration::RegisterForPDIPC<PDIPCCloth>("cloth");
const bool pdipc_cloth_deleter_registered = TypeErasure::RegisterForDeleter<PDIPCCloth>("cloth");
const bool pdipc_cloth_render_object_registered = CasterRegistration::RegisterForCaster<Renderable, PDIPCCloth>("cloth");