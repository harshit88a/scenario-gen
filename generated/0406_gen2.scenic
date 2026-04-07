param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param EGO_SPEED = Range(8, 12)
param PED_SPEED = 2.0
param PED_WALK_DURATION = 2.0

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior ParkedBehavior():
    take SetHandBrakeAction(True)

behavior CrossAndFreeze(ego, speed, duration):
    do CrossingBehavior(lambda: ego, speed, 300) for duration seconds
    terminate

from scenic.simulators.carla.behaviors import FollowLaneBehavior, CrossingBehavior
from scenic.simulators.carla.actions import SetHandBrakeAction

PARKED_MODEL = "vehicle.nissan.patrol"
PED_MODEL = "walker.pedestrian.0001"

initLane = Uniform(*network.lanes)
ego_spawn_pt = new OrientedPoint on initLane.centerline
parked_spawn_pt = new OrientedPoint right of ego_spawn_pt by 3.5

require (distance from ego_spawn_pt to intersection) > 50

param PARKED_DIST = Range(25, 35)
param PED_DIST_FROM_CAR = 4.0

parked_spot = new OrientedPoint following roadDirection from parked_spawn_pt for globalParameters.PARKED_DIST

ped_ref_spot = new OrientedPoint followingroadDirection from parked_spot for globalParameters.PED_DIST_FROM_CAR
ped_spawn_pt = new OrientedPoint left of ped_ref_spot by 0.5

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

parked_car = new Car at parked_spot offset by (0, 0 , 0.5),
    with bluePrint PARKED_MODEL,
    with behavior ParkedBehavior()
