#include "Object.hpp"
#include "Data/GridData.hpp"
#include "Coordinate/GridCoordinate.hpp"
#include "EnergyModel/GridEnergyModel.hpp"
#include "MassModel/MassModel.hpp"
#include "Render/GridShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"
#include "ExternalForce/SampledObjectGravity.hpp"
#include "ExternalForce/SampledObjectFixtureForce.hpp"

using Grid = ConcreteObject<GridData, GridData, GridCoordinate, SampledObjectMassModel, GridEnergyModel, GridShape, NullCollisionShape>;

