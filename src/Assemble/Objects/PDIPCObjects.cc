#include "Data/PDClothData.hpp"

#include "Coordinate/Coordinate.hpp"
#include "Coordinate/CoordinateAdapter.hpp"

#include "MassModel/MassModel.hpp"
#include "MassModel/MassModelAdapter.hpp"

#include "Object.hpp"

#include "PDEnergyModel/PDClothEnergyModel.hpp"
#include "PDEnergyModel/PDNullEnergyModel.hpp"

#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"

#include "Render/RenderShape.hpp"
#include "Render/RenderShapeAdapter.hpp"
#include "Render/RenderInterface.hpp"

#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionShape/CollisionShapeAdapter.hpp"

#include "Collision/CollisionShape/ImplicitShape/ImplicitSphere.hpp"

#include "TimeStepper/PDIPC.hpp"
#include "TimeStepper/PositionBasedPDIPC.hpp"

class PDIPCCloth :
	public PDClothData,
    public CoordinateAdapter<BasicCoordinate, PDIPCCloth>,
    public MassModelAdapter<SampledObjectMassModel, PDIPCCloth>,
    public PDClothEnergyModel<PDIPCCloth, ParallelType::kNone>,
    public ExternalForceContainerAdapter<PDClothData, PDIPCCloth>,
    public RenderShapeAdapter<SampledRenderShape, PDIPCCloth>,
	public CollisionShapeAdapter<SampledCollisionShape, PDIPCCloth> {

public:
	PDIPCCloth(const json& config):
        PDClothData(PDClothData::CreateFromFile(config)),
        PDClothEnergyModel(PDClothEnergyModel::CreateFromConfig(config["energy"])),
        ExternalForceContainerAdapter(ExternalForceContainer<PDClothData>::CreateFromConfig(config["external-forces"])),
        RenderShapeAdapter(SampledRenderShape::CreateFromConfig(config["render"])),
		CollisionShapeAdapter(config["collision"]){
        PDClothEnergyModel::Initialize();
		CollisionShapeAdapter::Initialize();
    }
};

const bool sampled_gravity_registered = ExternalForceRegistration::RegisterExternalForce<SampledObjectGravity, PDClothData>("gravity");
const bool pdipc_cloth_registered = CreatorRegistration::RegisterForCreator<PDIPCCloth>("cloth");
const bool pdipc_cloth_caster_registered = ObjectRegistration::RegisterForPDIPC<PDIPCCloth>("cloth");
const bool position_based_pdipc_cloth_caster_registered = ObjectRegistration::RegisterForPositionBasedPDIPC<PDIPCCloth>("cloth");
const bool pdipc_cloth_deleter_registered = TypeErasure::RegisterForDeleter<PDIPCCloth>("cloth");
const bool pdipc_cloth_render_object_registered = CasterRegistration::RegisterForCaster<Renderable, PDIPCCloth>("cloth");

class PDIPCFixedObject :
    public FixedSampledObjectData,
    public CoordinateAdapter<BasicCoordinate, PDIPCFixedObject>,
    public MassModelAdapter<SampledObjectMassModel, PDIPCFixedObject>,
    public PDNullEnergyModel,
    public ExternalForceContainerAdapter<FixedSampledObjectData, PDIPCFixedObject>,
    public RenderShapeAdapter<SampledRenderShape, PDIPCFixedObject>,
    public CollisionShapeAdapter<SampledCollisionShape, PDIPCFixedObject> {
public:
    PDIPCFixedObject(const json& config) :
		FixedSampledObjectData(FixedSampledObjectData::CreateFromConfig(config)),
        ExternalForceContainerAdapter(ExternalForceContainer<FixedSampledObjectData>::CreateFromConfig(config["external-forces"])),
        RenderShapeAdapter(SampledRenderShape::CreateFromConfig(config["render"])),
        CollisionShapeAdapter(config["collision"]) {
        CollisionShapeAdapter::Initialize();
    }
};

const bool pdipc_fixed_object_registered = CreatorRegistration::RegisterForCreator<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_caster_registered = ObjectRegistration::RegisterForPDIPC<PDIPCFixedObject>("fixed-object");
const bool position_based_pdipc_fixed_object_caster_registered = ObjectRegistration::RegisterForPositionBasedPDIPC<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_deleter_registered = TypeErasure::RegisterForDeleter<PDIPCFixedObject>("fixed-object");
const bool pdipc_fixed_object_render_object_registered = CasterRegistration::RegisterForCaster<Renderable, PDIPCFixedObject>("fixed-object");
