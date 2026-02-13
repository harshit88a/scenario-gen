# Scenario: Ego vehicle makes a right turn at an intersection and must yield when pedestrian crosses the crosswalk.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Pedestrian's crossing response using standard Carla behaviors. The adversarial pedestrian enters the crosswalk on the destination road, requiring the ego vehicle to yield and stop mid-turn.

from scenic.simulators.carla.behaviors import CrossingBehavior
from scenic.domains.driving.roads import ManeuverType

param EGO_SPEED = 9
param PED_SPEED = 1.5
param TRIGGER_DIST = 12

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. GEOMETRY
# Filters the road network for valid intersections and selects a right-turn maneuver to construct the ego vehicle's path.

intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

right_turns = filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*right_turns)

egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is on the sidewalk or corner to the front-right of the ego vehicle.

param PED_OFFSET = Range(4.0, 6.0)

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

require 20 <= (distance from ego to egoInitLane.centerline[-1]) <= 25

turn_point = new OrientedPoint at egoInitLane.centerline[-1]
ped_spawn = new OrientedPoint right of turn_point by globalParameters.PED_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to turn_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

require ped not in network.drivableRegion

terminate when (distance to egoSpawnPt) > 50