# Scenario: Ego vehicle drives straight; adversarial pedestrian appears from behind a parked car on the right and suddenly stops in the lane.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian suddenly dashes into the road from behind a parked vehicle and comes to an abrupt stop in the ego vehicle's lane.

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


## 3. GEOMETRY
# A straight road with a parking lane or shoulder on the right side.


from scenic.simulators.carla.behaviors import FollowLaneBehavior, CrossingBehavior
from scenic.simulators.carla.actions import SetHandBrakeAction

PARKED_MODEL = "vehicle.nissan.patrol"
PED_MODEL = "walker.pedestrian.0001"

initLane = Uniform(*network.lanes)
ego_spawn_pt = new OrientedPoint on initLane.centerline
parked_spawn_pt = new OrientedPoint right of ego_spawn_pt by 3.5

require (distance from ego_spawn_pt to intersection) > 50


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is initially positioned on the right, hidden behind a parked car in the ego vehicle's front-right periphery.

param PARKED_DIST = Range(25, 35)
param PED_DIST_FROM_CAR = 4.0 

parked_spot = new OrientedPoint following roadDirection from parked_spawn_pt for globalParameters.PARKED_DIST

ped_ref_spot = new OrientedPoint following roadDirection from parked_spot for globalParameters.PED_DIST_FROM_CAR
ped_spawn_pt = new OrientedPoint right of ped_ref_spot by 0.5

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

parked_car = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint PARKED_MODEL,
    with behavior ParkedBehavior()