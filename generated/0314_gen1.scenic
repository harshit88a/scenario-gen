# Scenario: Ego vehicle is driving on a straight road; the adversarial pedestrian crosses from the left at a sampled trigger distance while the ego travels at standard speed.

# 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian triggers at a sampled distance and walks at a fixed moderate speed toward the ego's path.

from scenic.domains.driving.roads import ManeuverType
from scenic.simulators.carla.behaviors import CrossingBehavior

param EGO_SPEED = 10
param PED_SPEED = 1.8
param TRIGGER_DIST = Range(15, 25)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

# 3. ROAD GEOMETRY
# A straight road section with no intersection within view range to keep the scenario focused on lane traversal.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial pedestrian is placed at a sampled forward distance relative to the ego's position on the left shoulder or front-left edge.

param START_DISTANCE = Range(20, 35)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

adv_spot_road = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

ped_spawn = new OrientedPoint left of adv_spot_road by 3.5

ped = new Pedestrian at ped_spawn offset by (0, 0 , 1),
    facing 90 deg relative to adv_spot_road.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, ManeuverType.CROSS_WALK)

require ped not in network.drivableRegion