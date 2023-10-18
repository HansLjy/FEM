#include "Data/PDClothData.hpp"

#include "Coordinate/Coordinate.hpp"
#include "Coordinate/CoordinateAdapter.hpp"

#include "MassModel/MassModel.hpp"
#include "MassModel/MassModelAdapter.hpp"

#include "Object.hpp"

#include "PDEnergyModel/PDClothEnergyModel.hpp"
#include "PDEnergyModel/PDNullEnergyModel.hpp"
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

class PDIPCFixedObject :
    public FixedObjectData,
    public CoordinateAdapter<BasicCoordinate, PDIPCFixedObject>,
    public MassModelAdapter<NullMassModel, PDIPCFixedObject>,
    public PDEnergyModelAdapter<PDNullEnergyModel, PDIPCFixedObject>,
    public ExternalForceContainerAdapter<FixedObjectData, PDIPCFixedObject>,
    public RenderShapeAdapter<FixedRenderShape, PDIPCFixedObject>,
    public CollisionShapeAdapter<FixedCollisionShape, PDIPCFixedObject> {
public:
    PDIPCFixedObject(const json& config) :
        PDEnergyModelAdapter({}),
        ExternalForceContainerAdapter(ExternalForceContainer<FixedObjectData>::CreateFromConfig(config["external-forces"])),
        RenderShapeAdapter(FixedRenderShape::CreateFromConfig(config["render"])),
        CollisionShapeAdapter(FixedCollisionShape::CreateFromConfig(config["collision"])) {
        CollisionShapeAdapter::Initialize();
    }

};

const bool pdipc_fixed_object_registered = CreatorRegistration::RegisterForCreator<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_caster_registered = ObjectRegistration::RegisterForPDIPC<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_deleter_registered = TypeErasure::RegisterForDeleter<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_render_object_registered = CasterRegistration::RegisterForCaster<Renderable, PDIPCFixedObject>("fixed-object");
