# Scenario: Ego vehicle makes a left turn at an intersection and must suddenly stop to avoid collision when pedestrian crosses the crosswalk.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Handles the pedestrian's reactive crossing behavior. The adversarial pedestrian enters the crosswalk while the ego vehicle is mid-turn, forcing the ego to perform an emergency stop.
 
from scenic.domains.driving.roads import ManeuverType
from scenic.simulators.carla.behaviors import CrossingBehavior

param EGO_SPEED = 10
param PED_SPEED = 1.5
param TRIGGER_DIST = 15

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. GEOMETRY
# An intersection (3-way or 4-way) with marked crosswalks.

intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

left_turns = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*left_turns)

egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is at the corner or on the sidewalk of the destination road, positioned to the front-left of the ego vehicle.

param PED_OFFSET = Range(3.0, 5.0)

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

exit_lane_start = egoManeuver.endLane.centerline[0]
exit_point = new OrientedPoint at exit_lane_start

ped_spawn = new OrientedPoint right of exit_point by globalParameters.PED_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to exit_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

require ped not in network.drivableRegion

terminate when (distance to egoSpawnPt) > 70