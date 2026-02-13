# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 10
param ADV_SPEED = Range(20, 25)
param GAP = Range(15, 20)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

# 3. ROAD GEOMETRY
ADV_MODEL = "vehicle.tesla.model3"

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 100

checkPt = new OrientedPoint left of spawnPt by 3.5
targetSection = network.laneSectionAt(checkPt)

require targetSection is not None
require targetSection.lane != initLane

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

adv_spawn_pt = new OrientedPoint following roadDirection from checkPt for -globalParameters.GAP

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior()

terminate when (distance to spawnPt) > 100