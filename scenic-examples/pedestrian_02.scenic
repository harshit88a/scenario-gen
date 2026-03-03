# Scenario: Ego vehicle must suddenly stop to avoid collision when pedestrian crosses the road unexpectedly.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian unexpectedly crosses the road using standard pedestrian crossing behavior.

from scenic.simulators.carla.behaviors import CrossingBehavior

param EGO_SPEED = 10
param PED_SPEED = 2.0
param TRIGGER_DIST = 15

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 3. GEOMETRY
# A straight road.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 50

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is initially on the sidewalk or road shoulder, positioned to the front-right or front-left of the ego vehicle.

param START_DISTANCE = Range(20, 30)
param LATERAL_OFFSET = Range(3.5, 4.5)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

adv_spot_road = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE
ped_spawn = new OrientedPoint right of adv_spot_road by globalParameters.LATERAL_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to adv_spot_road.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

require ped not in network.drivableRegion