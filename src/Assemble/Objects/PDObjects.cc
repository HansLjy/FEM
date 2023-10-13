#include "Data/PDClothData.hpp"

#include "Coordinate/Coordinate.hpp"
#include "Coordinate/CoordinateAdapter.hpp"

#include "MassModel/MassModel.hpp"
#include "MassModel/MassModelAdapter.hpp"

#include "PDEnergyModel/PDClothEnergyModel.hpp"
#include "PDEnergyModel/PDEnergyAdapter.hpp"

#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"

#include "Render/RenderShape.hpp"
#include "Render/RenderShapeAdapter.hpp"
#include "Render/RenderInterface.hpp"

#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionShape/CollisionShapeAdapter.hpp"

#include "TimeStepper/ProjectiveDynamics.hpp"

class PDCloth :
    public PDClothData,
    public CoordinateAdapter<BasicCoordinate, PDCloth>,
    public MassModelAdapter<SampledObjectMassModel, PDCloth>,
    public PDEnergyModelAdapter<PDClothEnergyModel, PDCloth>,
    public ExternalForceContainerAdapter<PDClothData, PDCloth>,
    public RenderShapeAdapter<SampledRenderShape, PDCloth>,
	public CollisionShapeAdapter<SampledCollisionShape, PDCloth> {
public:
    PDCloth(const json& config):
        PDClothData(PDClothData::CreateFromFile(config)),
        PDEnergyModelAdapter(PDClothEnergyModel::CreateFromConfig(config["energy"])),
        ExternalForceContainerAdapter(ExternalForceContainer<PDClothData>::CreateFromConfig(config["external-forces"])),
        RenderShapeAdapter(SampledRenderShape::CreateFromConfig(config["render"])),
		CollisionShapeAdapter(config["collision"]){
		// PDEnergyModelAdapter::Initialize();
		CollisionShapeAdapter::Initialize();
    }

};

const bool sampled_gravity_registered = ExternalForceRegistration::RegisterExternalForce<SampledObjectGravity, PDClothData>("gravity");
const bool pd_cloth_registered = CreatorRegistration::RegisterForCreator<PDCloth>("cloth");
const bool pd_cloth_caster_registered = ObjectRegistration::RegisterForPD<PDCloth>("cloth");
const bool pd_cloth_deleter_registered = TypeErasure::RegisterForDeleter<PDCloth>("cloth");
const bool pd_cloth_render_object_registered = CasterRegistration::RegisterForCaster<Renderable, PDCloth>("cloth");
