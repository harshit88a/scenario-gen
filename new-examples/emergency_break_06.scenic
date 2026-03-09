# Scenario: A slow-moving lead vehicle is followed closely by the ego over a long straight stretch until the lead car encounters a distant static box and brakes, giving ego marginally more warning time but creating a dangerous compressed following scenario at low speed differential.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Lead car crawls at a sampled low speed and applies full brakes at a sampled close trigger distance
param LEAD_SPEED = Range(4, 8)
param OBSTACLE_BRAKE_DIST = Range(8, 12)
param BRAKE_FORCE = 1.0

behavior LeadCarBehavior(target_obstacle, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    while True:
        take SetBrakeAction(brake_intensity)

# 3. ROAD GEOMETRY
# Ego drives faster than the lead car, gradually closing the gap toward the slow-moving vehicle
EGO_SPEED = 14

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Lead car starts close ahead of ego; obstacle is placed at a far sampled distance to extend scenario duration
param DIST_EGO_TO_LEAD = Range(8, 12)
param DIST_LEAD_TO_OBSTACLE = Range(45, 60)

lead_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

obs_spot = new OrientedPoint following roadDirection from lead_spot for globalParameters.DIST_LEAD_TO_OBSTACLE

obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.creasedbox01"

leadCar = new Car at lead_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

terminate when ego.speed < 0.1 and (distance to leadCar) < 5