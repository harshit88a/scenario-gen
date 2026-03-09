# Scenario: The adversarial leading car drives slowly to lure the ego into closing the gap, then abruptly surges to a high sampled speed, creating a sudden and dangerous speed differential.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Adversarial car crawls at a sampled slow speed until ego closes in, then surges to a sampled high speed
param ADV_SLOW_SPEED = Range(3, 7)
param ADV_FAST_SPEED = Range(28, 40)
param ADV_DISTANCE = Range(10, 18)

behavior AdvBehavior(target_ego):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SLOW_SPEED) until \
        (distance from target_ego to self) < globalParameters.ADV_DISTANCE
    do FollowLaneBehavior(target_speed=globalParameters.ADV_FAST_SPEED)

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane, at a sampled position far from any intersection
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Adversarial car spawns at a sampled forward distance ahead of ego
param GEO_Y_DISTANCE = Range(12, 22)
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE
advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior AdvBehavior(ego)