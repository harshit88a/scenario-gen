# Scenario: A fast-moving lead vehicle travels well above the ego's speed and brakes sharply upon reaching a barrel obstacle at a sampled trigger distance, creating a compressed reaction window for ego.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Lead car samples a high speed and brakes hard at a sampled close trigger distance from the barrel
param LEAD_SPEED = Range(18, 28)
param OBSTACLE_BRAKE_DIST = Range(8, 14)
param BRAKE_FORCE = 1.0

behavior LeadCarBehavior(target_obstacle, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    while True:
        take SetBrakeAction(brake_intensity)

# 3. ROAD GEOMETRY
# Ego spawns at a standard cruising speed, well below the lead car's sampled speed
EGO_SPEED = 14

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Lead car spawns close behind ego's forward view; barrel obstacle is placed far ahead at a sampled distance
param DIST_EGO_TO_LEAD = Range(10, 16)
param DIST_LEAD_TO_OBSTACLE = Range(28, 45)

lead_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

obs_spot = new OrientedPoint following roadDirection from lead_spot for globalParameters.DIST_LEAD_TO_OBSTACLE

obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.barrel"

leadCar = new Car at lead_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

terminate when ego.speed < 0.1 and (distance to leadCar) < 5