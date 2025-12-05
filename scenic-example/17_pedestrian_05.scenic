# Scenario: Ego vehicle goes straight at an intersection and must yield when pedestrian crosses the crosswalk.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Explicitly import behaviors and road enums for Scenic 3.0.0
from scenic.simulators.carla.behaviors import CrossingBehavior
from scenic.domains.driving.roads import ManeuverType

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 10
param PED_SPEED = 1.5
param TRIGGER_DIST = 15

# Ego: Follows the specific straight trajectory
behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 1. Select a 3-way or 4-way intersection
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

# 2. Select a Straight maneuver from this intersection
# We access ManeuverType from the imported 'roads' module
straight_maneuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers)
egoManeuver = Uniform(*straight_maneuvers)

# 3. Define the trajectory: Start Lane -> Connecting Lane (Intersection) -> End Lane
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 4. Create Ego spawn point
# Spawn on the centerline of the starting lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Requirement: Ensure Ego starts 20-25m back from the intersection entrance
# centerline[-1] represents the stop line at the intersection
require 20 <= (distance from ego to egoInitLane.centerline[-1]) <= 25

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param PED_OFFSET = Range(3.0, 5.0)

# 1. Identify the conflict zone (Far side crosswalk)
# For a straight maneuver, the conflict often happens at the exit of the intersection.
# egoManeuver.endLane.centerline[0] is the start of the road *after* the intersection.
exit_point = new OrientedPoint at egoManeuver.endLane.centerline[0]

# 2. Define Pedestrian spawn point
# Placed to the right of the exit point (sidewalk area on the far side)
ped_spawn = new OrientedPoint right of exit_point by globalParameters.PED_OFFSET

# 3. Create the Pedestrian
# Uses CrossingBehavior: (ego_agent, walking_speed, trigger_distance)
# - facing 90 deg relative to exit_point: Pedestrian faces the road (Left relative to their spawn side) to cross.
# - offset z by 1.0: Prevents ground clipping.
ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to exit_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

# Requirement: Pedestrian must start on the sidewalk (not drivable region)
require ped not in network.drivableRegion

# Termination conditions
terminate when (distance to egoSpawnPt) > 70