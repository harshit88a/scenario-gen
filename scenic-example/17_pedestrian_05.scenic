# Scenario: Ego vehicle goes straight at an intersection and must yield when pedestrian crosses the crosswalk.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Ego vehicle's straight-line traversal and the pedestrian's triggered crossing behavior at the intersection exit. The adversarial pedestrian enters the crosswalk perpendicular to the ego vehicle’s path, requiring the ego to yield or stop to avoid a collision.

from scenic.simulators.carla.behaviors import CrossingBehavior
from scenic.domains.driving.roads import ManeuverType

param EGO_SPEED = 10
param PED_SPEED = 1.5
param TRIGGER_DIST = 15

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. GEOMETRY
# Selects an intersection and filters for straight-through maneuvers to establish the trajectory for the ego vehicle.

intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

straight_maneuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers)
egoManeuver = Uniform(*straight_maneuvers)

egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is on the sidewalk at the corner, positioned to the front-right or front-left of the ego vehicle.

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

require 20 <= (distance from ego to egoInitLane.centerline[-1]) <= 25

exit_point = new OrientedPoint at egoManeuver.endLane.centerline[0]
ped_spawn = new OrientedPoint right of exit_point by globalParameters.PED_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to exit_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

require ped not in network.drivableRegion

terminate when (distance to egoSpawnPt) > 70