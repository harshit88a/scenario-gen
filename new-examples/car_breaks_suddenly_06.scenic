# Scenario: An adversarial vehicle spawns one lane-width to the left of ego and ahead,  follows its adjacent lane at a sampled speed, then steers sharply into the ego's lane simulating an aggressive and dangerous cut-in maneuver.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Adversarial vehicle follows its adjacent lane at sampled speed, then steers sharply into ego's lane
param ADV_SPEED = Range(15, 25)
param ADV_DISTANCE = Range(10, 20)
param STEER_INTENSITY = 0.5

behavior AdvBehavior(target_ego, steer_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance from target_ego to self) < globalParameters.ADV_DISTANCE
    while True:
        take SetSteerAction(steer_intensity)

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane far from any intersection
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Adversarial car spawns at a sampled forward distance and one lane-width to the left of ego
param GEO_Y_DISTANCE = Range(15, 30)
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE
advAgent = new Car at adv_spot offset by (-3.5, 0, 0.5),
    with behavior AdvBehavior(ego, globalParameters.STEER_INTENSITY)