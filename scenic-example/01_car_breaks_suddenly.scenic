# Scenario: Ego drives straight; Leading car brakes suddenly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param ADV_SPEED = Range(0, 10)
param ADV_DISTANCE = Range(10, 20) 
param BRAKE_FORCE = Range(0.5, 1)  

# Adversary Behavior: Drive -> Wait for Ego -> Brake
behavior AdvBehavior(target_ego, brake_intensity):
    # Drive normally until Ego is close
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance from target_ego to self) < globalParameters.ADV_DISTANCE
    
    # Then brake hard forever
    while True:
        take SetBrakeAction(brake_intensity)


## 3. GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Find a valid lane
lane = Uniform(*network.lanes)

# Create the Spawn Point 
EgoSpawnPt = new OrientedPoint on lane.centerline

# Create Ego
# We give it a speed of 15 m/s so it is faster than the Adversary (0-10 m/s)
# This ensures it "approaches" the adversary to trigger the brake logic.
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=15)

# Requirement: Ensure we are not spawning inside an intersection
require (distance to intersection) > 50


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param GEO_Y_DISTANCE = Range(15, 30) 

# Step 1: Create the "Target Spot" on the road explicitly
adv_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.GEO_Y_DISTANCE

# Step 2: Spawn the car AT that spot
advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior AdvBehavior(ego, globalParameters.BRAKE_FORCE)