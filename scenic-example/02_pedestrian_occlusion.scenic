# Scenario 02: Pedestrian Crossing from Behind Occlusion - A parked truck occludes the view of a pedestrian who 
# jaywalks into the ego vehicle's path when the ego gets close.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Parameters for the adversary search
param PED_SPEED = Range(1.5, 3.0)       # Walking speed
param TRIGGER_DIST = Range(10, 18)      # Distance from Ego to start walking

# Custom helper behavior to handle walking actions explicitly
# This avoids "unexpected keyword argument" errors in different library versions
behavior CrossingWalk(target_speed):
    while True:
        # Continuously set direction and speed to ensure movement
        take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(target_speed)

# Main Pedestrian Behavior
behavior PedestrianOcclusionBehavior(target_ego):
    # State 1: Wait while hidden behind the truck
    while (distance from target_ego to self) > globalParameters.TRIGGER_DIST:
        wait
    
    # State 2: Cross the street using our custom walker behavior
    do CrossingWalk(target_speed=globalParameters.PED_SPEED)

# Behavior for the Truck: Simply stay parked
behavior ParkedBehavior():
    take SetBrakeAction(1.0)
    take SetHandBrakeAction(True)


## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.tesla.model3"

# Find a valid lane
lane = Uniform(*network.lanes)

# Create the Ego Spawn Point
EgoSpawnPt = new OrientedPoint on lane.centerline

# Spawn Ego
# Speed is set to 10 m/s (36 km/h)
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=10)

# Requirement: Ensure clean geometry (not in intersection)
require (distance to intersection) > 40


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Distance of the truck ahead of the ego
param OCCLUDER_DIST = Range(20, 30) 
param SIDE_OFFSET = -3.5 

# Step 1: Create the spot for the Parked Truck (The Occluder)
truck_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OCCLUDER_DIST

# Step 2: Spawn the Truck
occluder_truck = new Truck at truck_spot offset by (globalParameters.SIDE_OFFSET, 0, 0.5),
    with behavior ParkedBehavior()

# Step 3: Spawn the Pedestrian relative to the Truck
# We define the orientation in the OrientedPoint using 'facing'.
# (-0.5, 4.0) places the ped slightly inside the truck's line and ahead of it.
# 'facing 90 deg relative to occluder_truck' rotates the point Left (towards the road).
ped_spawn = new OrientedPoint at (-0.5, 4.0) relative to occluder_truck,
    facing 90 deg relative to occluder_truck

bad_pedestrian = new Pedestrian at ped_spawn,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego)

# Requirement: Ensure the pedestrian is valid and not colliding immediately
require (distance from bad_pedestrian to occluder_truck) > 1