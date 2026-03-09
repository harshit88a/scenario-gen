# Scenario: Ego drives straight and encounters a fixed arrangement of three static barrels placed at
# precise known distances ahead, simulating a fully deterministic road obstruction with no randomness.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# All three barrel props are fully static hazards; they hold position for the entire scenario duration
param DEBRIS_START_DIST = 18
param DEBRIS_GAP = 5

behavior StaticObjectBehavior():
    while True:
        wait

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane section well away from any intersection at a fixed cruising speed
EGO_SPEED = 12

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Three barrels are placed at fixed distances along the lane centerline directly in the ego's path
debris_spot_1 = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris1 = new Prop at debris_spot_1,
    with blueprint "static.prop.barrel",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_2 = new OrientedPoint following roadDirection from debris_spot_1 for globalParameters.DEBRIS_GAP

debris2 = new Prop at debris_spot_2,
    with blueprint "static.prop.barrel",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_3 = new OrientedPoint following roadDirection from debris_spot_2 for globalParameters.DEBRIS_GAP

debris3 = new Prop at debris_spot_3,
    with blueprint "static.prop.barrel",
    with physics True,
    with behavior StaticObjectBehavior()

terminate when (distance from ego to EgoSpawnPt) > 70