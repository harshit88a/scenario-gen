# Scenario: Ego vehicle must suddenly stop to avoid collision when pedestrian crosses the road unexpectedly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Explicitly import the standard CrossingBehavior
from scenic.simulators.carla.behaviors import CrossingBehavior

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 10
param PED_SPEED = 2.0
param TRIGGER_DIST = 15

# Ego: Simply follows the lane at a set speed
behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Pick a lane from the network
initLane = Uniform(*network.lanes)

# Create a spawn point on this lane
spawnPt = new OrientedPoint on initLane.centerline

# Create Ego at the spawn point
# Offset z by 0.5 to prevent ground clipping
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Requirement: Avoid spawning too close to intersections
require (distance to intersection) > 50

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param START_DISTANCE = Range(20, 30)
param LATERAL_OFFSET = Range(3.5, 4.5)

# 1. Find a reference point ahead of the ego on the road center
adv_spot_road = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

# 2. Define the pedestrian spawn point to the right of the road (sidewalk)
ped_spawn = new OrientedPoint right of adv_spot_road by globalParameters.LATERAL_OFFSET

# 3. Create the Pedestrian
# We use the built-in 'CrossingBehavior' which accepts (ego, speed, threshold).
# - ego: The agent to wait for.
# - speed: Walking speed.
# - threshold: Distance to ego to trigger crossing.
# Offset z by 1.0 to ensure the pedestrian doesn't spawn in the ground.
ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to adv_spot_road.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

# Requirement: Pedestrian must start off the drivable lane
require ped not in network.drivableRegion