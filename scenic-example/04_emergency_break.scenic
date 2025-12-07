# Scenario: Lead Vehicle Emergency Brake due to Obstacle - The leading vehicle decelerates suddenly due to a trash bin, forcing the ego-vehicle to perform an emergency brake.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param LEAD_SPEED = Range(10, 12)
param OBSTACLE_BRAKE_DIST = Range(10, 15)
param BRAKE_FORCE = 1.0

# Lead Car Behavior: Drive -> Detect Obstacle -> Brake
behavior LeadCarBehavior(target_obstacle, brake_intensity):
    # Drive normally until Obstacle is close
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    
    # Then brake hard
    while True:
        take SetBrakeAction(brake_intensity)


## 3. GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 12

# Find a valid lane
lane = Uniform(*network.lanes)

# Create the Ego Spawn Point 
EgoSpawnPt = new OrientedPoint on lane.centerline

# Create Ego
# Spawning Ego first as the reference point
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

# Requirement: Ensure we are not spawning near an intersection
require (distance to intersection) > 80


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param DIST_EGO_TO_LEAD = Range(10, 15)
param DIST_LEAD_TO_OBSTACLE = Range(30, 40)

# Step 1: Create the "Lead Car Spot" relative to Ego
lead_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

# Step 2: Create the "Obstacle Spot" relative to the Lead Car spot
# We define this first so we can spawn the obstacle and pass it to the Lead Car behavior
obs_spot = new OrientedPoint following roadDirection from lead_spot for globalParameters.DIST_LEAD_TO_OBSTACLE

# Step 3: Spawn the Obstacle (Trash Can)
# CORRECTED BLUEPRINT ID: static.prop.trashcan01
obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.trashcan01"

# Step 4: Spawn the Lead Car (Adversary) at the lead spot
leadCar = new Car at lead_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

# Termination: End when ego stops or gets too close (collision/unsafe)
terminate when ego.speed < 0.1 and (distance to leadCar) < 5