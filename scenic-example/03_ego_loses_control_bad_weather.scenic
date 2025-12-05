# Scenario: Control Loss Due to Road Conditions - The ego-vehicle encounters unexpected debris (crushed boxes)
# on the road, simulating bad conditions that force a reaction.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Parameters for the adversary generation
param DEBRIS_START_DIST = Range(15, 20)
param DEBRIS_GAP = Range(6, 10) 

# Behavior for the Debris (Static props)
behavior StaticObjectBehavior():
    while True:
        wait

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 10

# Select a valid lane
lane = Uniform(*network.lanes)

# Create the Ego Spawn Point
EgoSpawnPt = new OrientedPoint on lane.centerline

# Spawn Ego
# FIX: Increased Z-offset to 1.0 to prevent ground collision (clipping) failure
ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

# Requirement: Ensure clean geometry (not in intersection)
require (distance to intersection) > 50

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# We spawn 'static.prop.creasedbox01' to simulate debris.

# Location for Debris 1
debris_spot_1 = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris1 = new Prop at debris_spot_1,
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

# Location for Debris 2 (relative to Debris 1)
debris_spot_2 = new OrientedPoint following roadDirection from debris_spot_1 for globalParameters.DEBRIS_GAP

debris2 = new Prop at debris_spot_2 offset by (Range(-0.5, 0.5), 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

# Location for Debris 3 (relative to Debris 2)
debris_spot_3 = new OrientedPoint following roadDirection from debris_spot_2 for globalParameters.DEBRIS_GAP

debris3 = new Prop at debris_spot_3 offset by (Range(-0.5, 0.5), 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

# FIX: Termination Condition
# We wait until the ego has traveled significantly past the debris area.
# Approx debris area ends at 20 + 10 + 10 = 40m. We terminate at 70m.
terminate when (distance from ego to EgoSpawnPt) > 70