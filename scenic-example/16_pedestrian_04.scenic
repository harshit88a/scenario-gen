# Scenario: Ego vehicle makes a right turn at an intersection and must yield when pedestrian crosses the crosswalk.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Explicitly import behaviors and road enums for Scenic 3.0.0
from scenic.simulators.carla.behaviors import CrossingBehavior
from scenic.domains.driving.roads import ManeuverType

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 9
param PED_SPEED = 1.5
param TRIGGER_DIST = 12

# Ego: Follows the specific turn trajectory
behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 1. Select a 3-way or 4-way intersection
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

# 2. Select a Right Turn maneuver from this intersection
# We access ManeuverType from the imported 'roads' module
right_turns = filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*right_turns)

# 3. Define the trajectory: Start Lane -> Connecting Lane (Turn) -> End Lane
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 4. Create Ego spawn point on the start lane
# We use a range to randomize the start distance slightly, similar to the legacy code
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Requirement: Ensure Ego starts 20-25m back from the intersection (end of start lane)
# The end of the centerline [-1] represents the stop line/entrance to the intersection
require 20 <= (distance from ego to egoInitLane.centerline[-1]) <= 25

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param PED_OFFSET = Range(4.0, 6.0)

# 1. Identify the conflict zone
# For a right turn, the conflict is often at the entrance of the intersection 
# or the immediate exit. The legacy code placed it at the end of the start lane.
turn_point = new OrientedPoint at egoInitLane.centerline[-1]

# 2. Define Pedestrian spawn point
# Placed to the right of the turn point (sidewalk/corner)
ped_spawn = new OrientedPoint right of turn_point by globalParameters.PED_OFFSET

# 3. Create the Pedestrian
# Uses CrossingBehavior to cross the street when ego approaches
# Offset z by 1.0 to prevent ground clipping
ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to turn_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

# Requirement: Pedestrian must start on the sidewalk (not drivable region)
require ped not in network.drivableRegion

# Termination conditions
terminate when (distance to egoSpawnPt) > 50