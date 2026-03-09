# Scenario: A lead vehicle drives at a fixed speed ahead of ego and brakes hard upon reaching a static cone obstacle placed at a precisely known distance, creating a fully deterministic chain reaction.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Lead car drives at a fixed speed and applies full brakes the moment it closes within a fixed distance of the cone
param LEAD_SPEED = 11
param OBSTACLE_BRAKE_DIST = 12
param BRAKE_FORCE = 1.0

behavior LeadCarBehavior(target_obstacle, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    while True:
        take SetBrakeAction(brake_intensity)

# 3. ROAD GEOMETRY
# Ego spawns on a straight lane at a fixed cruising speed well away from any intersection
EGO_SPEED = 12

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Lead car and construction cone are placed at fixed distances directly ahead of ego along the lane
param DIST_EGO_TO_LEAD = 12
param DIST_LEAD_TO_OBSTACLE = 35

lead_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

obs_spot = new OrientedPoint following roadDirection from lead_spot for globalParameters.DIST_LEAD_TO_OBSTACLE

obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.constructioncone"

leadCar = new Car at lead_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

terminate when ego.speed < 0.1 and (distance to leadCar) < 5