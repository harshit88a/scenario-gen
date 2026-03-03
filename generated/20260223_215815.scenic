# SCENARIO: The ego vehicle is driving on a straight road; the adversarial pedestrian suddenly crosses the road from the right front and suddenly stops in front of the ego. 

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
from scenic.simulators.carla.behaviors import FollowLaneBehavior, CrossingBehavior

param EGO_SPEED = Range(8, 12)
param PED_SPEED = 2.0
param PED_WALK_DURATION = 2.5

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior CrossAndFreeze(ego, speed, duration):
    do CrossingBehavior(lambda: ego, speed, 300) for duration seconds
    terminate

# 3. ROAD GEOMETRY
initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param PED_DIST_AHEAD = Range(15, 20)
param PED_OFFSET_RIGHT = Range(3.5, 5.0)
PED_MODEL = "walker.pedestrian.0001"

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Calculate pedestrian spawn point: Ahead of ego, offset to the right
ped_base_pt = new OrientedPoint following roadDirection from spawnPt for globalParameters.PED_DIST_AHEAD
ped_spawn_pt = new OrientedPoint right of ped_base_pt by globalParameters.PED_OFFSET_RIGHT

ped = new Pedestrian at ped_spawn_pt offset by (0, 0, 1),
    facing 90 deg relative to spawnPt.heading,
    with blueprint PED_MODEL,
    with behavior CrossAndFreeze(ego, globalParameters.PED_SPEED, globalParameters.PED_WALK_DURATION)

require ped not in network.drivableRegion

terminate when (distance to spawnPt) > 60