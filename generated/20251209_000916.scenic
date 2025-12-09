# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Standard behaviors
from scenic.simulators.carla.behaviors import FollowLaneBehavior
from scenic.simulators.carla.actions import SetBrakeAction

# Constants
param EGO_SPEED = Range(10, 14)
param ADV_SPEED = Range(10, 14)
param BRAKE_TRIGGER_DISTANCE = Range(15, 20)
param BRAKE_INTENSITY = 1.0

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior AdversaryBehavior(speed, trigger_dist):
    # Drive normally until the ego gets too close
    do FollowLaneBehavior(target_speed=speed) until (distance to ego) < trigger_dist
    # Then, slam on the brakes
    take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

# 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# 1. Select a lane on the map
initLane = Uniform(*network.lanes)

# 2. Define a spawn point for the ego vehicle on that lane
ego_spawn_pt = new OrientedPoint on initLane.centerline

# 3. Requirement: Ensure the scenario starts on a straight road, far from any intersections
require (distance from ego_spawn_pt to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param INITIAL_SEPARATION = Range(25, 35)

# 1. Define the adversary's spawn point ahead of the ego on the same lane
adv_spawn_pt = new OrientedPoint following roadDirection from ego_spawn_pt for globalParameters.INITIAL_SEPARATION

# 2. Instantiate the Ego Vehicle
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# 3. Instantiate the Adversary Vehicle
adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED, globalParameters.BRAKE_TRIGGER_DISTANCE)

# Termination condition
terminate when (distance to ego_spawn_pt) > 150