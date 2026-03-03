# Scenario: Ego vehicle is driving on a straight road; the adversarial pedestrian suddenly crosses the road from the right front and suddenly stops in front of the ego vehicle.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian crosses from the right front and suddenly stops in the ego's path.

param EGO_SPEED = 10
param PED_SPEED = 1.8
param TRIGGER_DIST = 18
param STOP_DIST = 4

behavior PedestrianCrossAndStopBehavior(ped_speed, trigger_dist, stop_dist):
    while (distance from self to ego) > trigger_dist:
        wait
    while True:
        if (distance from self to ego) <= stop_dist:
            take SetWalkingSpeedAction(0)
            break
        else:
            take SetWalkingDirectionAction(self.heading)
            take SetWalkingSpeedAction(ped_speed)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 3. GEOMETRY
# A straight road section, away from intersections.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 60

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian spawns to the right front of the ego vehicle, positioned on the road shoulder / sidewalk, facing leftward to cross into ego's path.

param START_DISTANCE = Range(20, 28)
param LATERAL_OFFSET = Range(3.5, 4.5)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

adv_spot_road = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE
ped_spawn = new OrientedPoint right of adv_spot_road by globalParameters.LATERAL_OFFSET

ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to adv_spot_road.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossAndStopBehavior(globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST, globalParameters.STOP_DIST)

require ped not in network.drivableRegion