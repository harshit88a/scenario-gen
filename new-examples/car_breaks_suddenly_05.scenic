# Scenario: Ego approaches a fully stationary adversarial vehicle stalled ahead in its lane at a fixed close distance with no warning, demanding immediate emergency braking to avoid a collision.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Adversarial vehicle holds maximum brakes for the entire scenario duration, remaining fully stationary
param BRAKE_FORCE = 1.0

behavior AdvBehavior():
    while True:
        take SetBrakeAction(globalParameters.BRAKE_FORCE)

# 3. ROAD GEOMETRY
# Ego spawns on a straight road section at a fixed point well away from any intersection
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)
require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Stalled adversarial vehicle is placed at a fixed close distance directly ahead of ego
param GEO_Y_DISTANCE = 22
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE
advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior AdvBehavior()