# Scenario: Two vehicles are chained ahead of ego in the same lane; the front vehicle brakes hard on a static barrier, causing a cascading emergency braking event through the mid vehicle back to the ego.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Both the front and mid cars brake independently upon closing within a sampled distance of their forward target
param LEAD_SPEED = Range(10, 14)
param OBSTACLE_BRAKE_DIST = Range(10, 15)
param BRAKE_FORCE = 1.0

behavior LeadCarBehavior(target_obstacle, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    while True:
        take SetBrakeAction(brake_intensity)

# 3. ROAD GEOMETRY
# Ego drives at standard speed toward the two-car chain ahead on a straight lane
EGO_SPEED = 12

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Mid car is placed between ego and front car; front car reacts to obstacle; mid car reacts to front car
param DIST_EGO_TO_MID = Range(10, 15)
param DIST_MID_TO_FRONT = Range(12, 18)
param DIST_FRONT_TO_OBSTACLE = Range(28, 38)

mid_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_MID

front_spot = new OrientedPoint following roadDirection from mid_spot for globalParameters.DIST_MID_TO_FRONT

obs_spot = new OrientedPoint following roadDirection from front_spot for globalParameters.DIST_FRONT_TO_OBSTACLE

obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.trashcan01"

frontCar = new Car at front_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

midCar = new Car at mid_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(frontCar, globalParameters.BRAKE_FORCE)

terminate when ego.speed < 0.1 and (distance to midCar) < 5