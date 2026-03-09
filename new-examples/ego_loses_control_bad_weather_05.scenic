# Scenario: Four creased boxes are scattered at large sampled intervals far down the road, simulating
# debris that fell off a vehicle over a long stretch, giving the ego more reaction time but requiring
# sustained hazard awareness over a greater distance.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Debris is scattered at large sampled intervals with wide sampled lateral noise to simulate realistic scatter
param DEBRIS_START_DIST = Range(20, 30)
param DEBRIS_GAP = Range(12, 20)

behavior StaticObjectBehavior():
    while True:
        wait

# 3. ROAD GEOMETRY
# Ego drives at a standard speed across a long straight section to encounter spread-out debris
EGO_SPEED = 11

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Four boxes are chained at large sampled gaps with wide sampled lateral scatter across the full span
debris_spot_1 = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris1 = new Prop at debris_spot_1 offset by (Range(-1.0, 1.0), 0, 0),
    with blueprint "static.prop.creasedbox02",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_2 = new OrientedPoint following roadDirection from debris_spot_1 for globalParameters.DEBRIS_GAP

debris2 = new Prop at debris_spot_2 offset by (Range(-1.0, 1.0), 0, 0),
    with blueprint "static.prop.creasedbox02",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_3 = new OrientedPoint following roadDirection from debris_spot_2 for globalParameters.DEBRIS_GAP

debris3 = new Prop at debris_spot_3 offset by (Range(-1.0, 1.0), 0, 0),
    with blueprint "static.prop.creasedbox02",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_4 = new OrientedPoint following roadDirection from debris_spot_3 for globalParameters.DEBRIS_GAP

debris4 = new Prop at debris_spot_4 offset by (Range(-1.0, 1.0), 0, 0),
    with blueprint "static.prop.creasedbox02",
    with physics True,
    with behavior StaticObjectBehavior()

terminate when (distance from ego to EgoSpawnPt) > 100