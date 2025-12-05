# Scenario: Ego vehicle makes a left turn at an intersection and must suddenly stop to avoid collision when pedestrian crosses the crosswalk.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Correct Import for Scenic 3.0.0
from scenic.domains.driving.roads import ManeuverType
from scenic.simulators.carla.behaviors import CrossingBehavior

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 10
param PED_SPEED = 1.5
param TRIGGER_DIST = 15

# Ego behavior: Follows the computed trajectory for the left turn
behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 1. Select a 3-way or 4-way intersection
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersection = Uniform(*intersections)

# 2. Filter for Left Turn maneuvers at this intersection
# We access ManeuverType from the imported 'roads' module
left_turns = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*left_turns)

# 3. Define the trajectory: Start Lane -> Connecting Lane (Turn) -> End Lane
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 4. Create Ego spawn point
# Offset z by 0.5 to prevent ground clipping
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

ego = new Car at egoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param PED_OFFSET = Range(3.0, 5.0)

# 1. Identify the exit point of the turn (start of the destination lane)
exit_lane_start = egoManeuver.endLane.centerline[0]
exit_point = new OrientedPoint at exit_lane_start

# 2. Define Pedestrian spawn point to the right of the road (sidewalk area)
ped_spawn = new OrientedPoint right of exit_point by globalParameters.PED_OFFSET

# 3. Create Pedestrian
# Uses CrossingBehavior: (ego, speed, threshold)
# Offset z by 1.0 to ensure valid spawn on uneven sidewalk/terrain
ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to exit_point.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

# Requirement: Pedestrian must start on the sidewalk (not drivable region)
require ped not in network.drivableRegion

# Terminate when ego completes the turn and moves away
terminate when (distance to egoSpawnPt) > 70