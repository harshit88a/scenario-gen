# Scenario: Ego vehicle drives straight; adversarial pedestrian appears from behind a parked car on the right and suddenly stops in the lane.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Standard behaviors
from scenic.simulators.carla.behaviors import FollowLaneBehavior, CrossingBehavior
# Action for the parked car
from scenic.simulators.carla.actions import SetHandBrakeAction

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = Range(8, 12)
param PED_SPEED = 2.0
param PED_WALK_DURATION = 2.0

# Ego: Simply follows the lane
behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

# Parked Car: Uses HandBrake (Boolean) to stay stationary
behavior ParkedBehavior():
    take SetHandBrakeAction(True)

# Pedestrian: Walks using CrossingBehavior with a time limit
behavior CrossAndFreeze(ego, speed, duration):
    # FIX: Pass 'lambda: ego' because the error suggested CrossingBehavior calls the ego argument.
    # We use a large threshold (300) to ensure the pedestrian starts walking immediately regardless of ego distance.
    do CrossingBehavior(lambda: ego, speed, 300) for duration seconds
    
    # Upon timeout, terminate. 
    # In CARLA/Scenic, terminating the behavior usually leaves the agent with the last control state 
    # or zeroed controls depending on the bridge. Without SetSpeed, this is the safest stop.
    terminate

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
PARKED_MODEL = "vehicle.nissan.patrol"
PED_MODEL = "walker.pedestrian.0001"

# 1. Select a lane
initLane = Uniform(*network.lanes)

# 2. Create Ego Spawn Point
ego_spawn_pt = new OrientedPoint on initLane.centerline

# 3. Define Parked Car Position (Occluder)
# Located ahead of the Ego, shifted to the right (shoulder/parking zone)
parked_spawn_pt = new OrientedPoint right of ego_spawn_pt by 3.5

# Requirement: Ensure the spawn is not too close to an intersection
require (distance from ego_spawn_pt to intersection) > 50

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param PARKED_DIST = Range(25, 35)
param PED_DIST_FROM_CAR = 4.0 

# 1. Update Parked Car location longitudinally
parked_spot = new OrientedPoint following roadDirection from parked_spawn_pt for globalParameters.PARKED_DIST

# 2. Create Pedestrian Spawn Point
# The pedestrian spawns 'behind' the parked car (relative to Ego's view)
# We offset slightly right (0.5m) to ensure they start fully behind the car body
ped_ref_spot = new OrientedPoint following roadDirection from parked_spot for globalParameters.PED_DIST_FROM_CAR
ped_spawn_pt = new OrientedPoint right of ped_ref_spot by 0.5

# 3. Instantiate Agents

# Ego Vehicle
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# Parked Car (The Occluder)
parked_car = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint PARKED_MODEL,
    with behavior ParkedBehavior()

# Adversarial Pedestrian
# facing 90 deg