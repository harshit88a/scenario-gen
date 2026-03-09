# Scenario: Ego drives straight and encounters a slow-moving adversarial vehicle ahead that drives at a fixed low speed for a few seconds then suddenly applies full brakes, forcing the ego into an emergency stop.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Adversarial vehicle crawls at a fixed low speed, then applies full brakes after a fixed duration
param ADV_SPEED = 4
param BRAKE_FORCE = 1.0

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for 4 seconds
    while True:
        take SetBrakeAction(globalParameters.BRAKE_FORCE)

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane segment well away from any intersection
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Adversarial car is placed at a fixed close distance directly ahead of the ego
param GEO_Y_DISTANCE = 18
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE
advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior AdvBehavior()