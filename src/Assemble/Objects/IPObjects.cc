#include "Data/ClothData.hpp"
#include "Data/CurveData.hpp"
#include "Data/MassSpringData.hpp"

#include "EnergyModel/ClothEnergyModel.hpp"
#include "EnergyModel/CurveEnergyModel.hpp"
#include "EnergyModel/MassSpringEnergyModel.hpp"
#include "EnergyModel/NullEnergyModel.hpp"
#include "EnergyModel/EnergyModelAdapter.hpp"

#include "MassModel/MassModel.hpp"
#include "MassModel/MassModelAdapter.hpp"

#include "Coordinate/Coordinate.hpp"
#include "Coordinate/CoordinateAdapter.hpp"

#include "Render/RenderShape.hpp"
#include "Render/CurveShape.hpp"
#include "Render/RenderShapeAdapter.hpp"
#include "Render/RenderInterface.hpp"

#include "Collision/CollisionShape/CollisionShape.hpp"
#include "Collision/CollisionShape/CollisionShapeAdapter.hpp"

#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/SampledObjectFixtureForce.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"

#include "TimeStepper/IncrementalPotential.hpp"



class Cloth :
    public ClothData,
    public CoordinateAdapter<BasicCoordinate, Cloth>,
    public EnergyModelAdapter<ClothEnergyModel, Cloth>,
    public MassModelAdapter<SampledObjectMassModel, Cloth>,
    public ExternalForceContainerAdapter<ClothData, Cloth>,
    public RenderShapeAdapter<SampledRenderShape, Cloth> {

public:
    Cloth(const json& config) :
        ClothData(config),
        EnergyModelAdapter(config["energy"]),
        ExternalForceContainerAdapter(config["external-forces"]),
        RenderShapeAdapter(config["render"]) {}
};

const bool cloth_gravity_registered = RegisterExternalForce<SampledObjectGravity, ClothData>("gravity");
const bool cloth_fix_force_registered = RegisterExternalForce<SampledObjecFixtureForce, ClothData>("fixture-force");
const bool cloth_object_registered = RegisterForCreator<Cloth>("cloth");
const bool cloth_render_object_registered = RegisterForCaster<Renderable, Cloth>("cloth");
const bool cloth_registered = RegisterForIP<Cloth>("cloth");

class Curve :
    public CurveData,
    public CoordinateAdapter<BasicCoordinate, Curve>,
    public MassModelAdapter<SampledObjectMassModel, Curve>,
    public EnergyModelAdapter<CurveEnergyModel, Curve>,
    public ExternalForceContainerAdapter<CurveData, Curve>,
    public RenderShapeAdapter<CurveRenderShape, Curve> {

public:
    Curve(const json& config) :
        CurveData(config),
        EnergyModelAdapter(config["energy"]),
        ExternalForceContainerAdapter(config["external-forces"]),
        RenderShapeAdapter(config["render"]) {}
};

const bool curve_object_registered = RegisterForCreator<Curve>("curve");
const bool curve_registered = RegisterForIP<Curve>("curve");