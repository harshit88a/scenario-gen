param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

from scenic.domains.driving.roads import ManeuverType
from scenic.simulators.carla.behaviors import CrossingBehavior

param EGO_SPEED = 10
param PED_SPEED = 2.0
param TRIGGER_DIST = 18

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

straight_maneuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers)
egoManeuver = Uniform(*straight_maneuvers)

egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

require 20 <= (distance from ego to egoInitLane.centerline[-1]) <= 25

exit_point = new OrientedPoint at egoManeuver.endLane.centerline[0]
ped_spawn = new OrientedPoint right of exit_point by globalParameters.PED_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0.5, 0.5),
    facing 90 deg relative to exit_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

require ped not in network.drivableRegion

terminate when (distance to egoSpawnPt) > 70

