# Scenario: Ego approaches a slow-moving leading vehicle and changes lanes to evade.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial vehicle maintains a slow speed while the ego performs an evasive lane change

param EGO_SPEED = 15
param ADV_SPEED = 6
param DIST_THRESHOLD = 15

behavior SlowLeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to advAgent) < globalParameters.DIST_THRESHOLD
    
    do LaneChangeBehavior(laneSectionToSwitch=target_section, target_speed=globalParameters.EGO_SPEED)
    
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)



## 3. GEOMETRY
### Defining the lane segments and verifying the availability of a secondary lane for the ego maneuver

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

checkPt = new OrientedPoint left of spawnPt by 3.5
targetSection = network.laneSectionAt(checkPt)

# Ensures targetSection exists and is a different lane before proceeding
require targetSection is not None
require getattr(targetSection, 'lane', None) != initLane

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

require (distance to intersection) > 50



## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The adversarial agent is positioned at a specific distance ahead of the ego within the same lane

param START_DISTANCE = Range(20, 30)

adv_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior SlowLeadBehavior()

require (distance from advAgent to intersection) > 20