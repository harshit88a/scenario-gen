# Scenario: A fast adversarial vehicle spawns behind the ego and aggressively accelerates to close the gap, threatening a rear-end collision before slamming on its brakes at a sampled critical distance.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Adversarial vehicle charges at sampled high speed from behind until dangerously close, then brakes hard
param ADV_SPEED = Range(22, 35)
param ADV_DISTANCE = Range(5, 10)
param BRAKE_FORCE = Range(0.8, 1.0)

behavior AdvBehavior(target_ego, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance from target_ego to self) < globalParameters.ADV_DISTANCE
    while True:
        take SetBrakeAction(brake_intensity)

# 3. ROAD GEOMETRY
# Ego spawns ahead of the adversarial base point on a straight lane far from any intersection
param GEO_Y_DISTANCE = Range(25, 45)
lane = Uniform(*network.lanes)
advBasePt = new OrientedPoint on lane.centerline
EgoSpawnPt = new OrientedPoint following roadDirection from advBasePt for globalParameters.GEO_Y_DISTANCE
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Adversarial tailgater spawns at the lane base point, placing it directly behind the ego
advAgent = new Car at advBasePt offset by (0, 0, 0.5),
    with behavior AdvBehavior(ego, globalParameters.BRAKE_FORCE)