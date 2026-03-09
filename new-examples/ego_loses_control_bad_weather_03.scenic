# Scenario: Ego is forced to slalom through a zigzag chain of creased boxes staggered alternately left and right of the lane centerline, demanding continuous steering corrections at speed.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Debris props are static; alternating lateral offsets are sampled per simulation to vary the slalom width
param DEBRIS_START_DIST = Range(15, 22)
param DEBRIS_GAP = Range(7, 12)
param LATERAL_OFFSET = Range(0.6, 1.2)

behavior StaticObjectBehavior():
    while True:
        wait

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane, cruising at a moderately high speed toward the zigzag obstacle chain
EGO_SPEED = 14

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Boxes alternate left and right of centerline at sampled gaps and sampled lateral widths
debris_spot_1 = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris1 = new Prop at debris_spot_1 offset by (globalParameters.LATERAL_OFFSET, 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_2 = new OrientedPoint following roadDirection from debris_spot_1 for globalParameters.DEBRIS_GAP

debris2 = new Prop at debris_spot_2 offset by (-globalParameters.LATERAL_OFFSET, 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_3 = new OrientedPoint following roadDirection from debris_spot_2 for globalParameters.DEBRIS_GAP

debris3 = new Prop at debris_spot_3 offset by (globalParameters.LATERAL_OFFSET, 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

terminate when (distance from ego to EgoSpawnPt) > 70