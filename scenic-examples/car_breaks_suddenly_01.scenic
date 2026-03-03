# Scenario: Ego drives straight; Leading car brakes suddenly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial car suddenly breaks when the ego approaches
param ADV_SPEED = Range(0, 10)
param ADV_DISTANCE = Range(10, 20) 
param BRAKE_FORCE = Range(0.5, 1)  

behavior AdvBehavior(target_ego, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance from target_ego to self) < globalParameters.ADV_DISTANCE
    while True:
        take SetBrakeAction(brake_intensity)


## 3. GEOMETRY
### Ego agent drives in a straight road
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The adversarial car is in front of the ego.
param GEO_Y_DISTANCE = Range(15, 30) 
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE
advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior AdvBehavior(ego, globalParameters.BRAKE_FORCE)