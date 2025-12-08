# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Explicit Imports
from scenic.simulators.carla.behaviors import CrossingBehavior
from scenic.domains.driving.roads import ManeuverType

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 10
param PED_SPEED = 2.0
param TRIGGER_DIST = 15

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

# 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 1. Intersection and Maneuver Selection
# We filter for intersections that have at least one Left Turn.
intersections = filter(lambda i: any(m.type is ManeuverType.LEFT_TURN for m in i.maneuvers), network.intersections)
intersection = Uniform(*intersections)

left_turns = filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers)
egoManeuver = Uniform(*left_turns)

# 2. Define Trajectory
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Spawn Ego
# We pick a point on the start lane and ensure it's close enough to the intersection
# but not inside it.
spawnPt = new OrientedPoint on egoInitLane.centerline
require (distance from spawnPt to intersection) > 10
require (distance from spawnPt to intersection) < 40 

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# We define the pedestrian relative to the destination lane (where ego exits the turn).
dest_lane = egoManeuver.endLane

# dest_lane.centerline[0] is just a Vector (x,y). We must convert it to an OrientedPoint
# aligned with the road so we can use '.heading' and 'right of'.
conflict_pt = new OrientedPoint following roadDirection from dest_lane.centerline[0] for 0

# 5. Pedestrian Placement
# Spawn 4-6m to the RIGHT of the lane start.
param PED_LATERAL_OFFSET = Range(4.0, 6.0)

ped_spawn = new OrientedPoint right of conflict_pt by globalParameters.PED_LATERAL_OFFSET

# Spawn Pedestrian
ped = new Pedestrian at ped_spawn offset by (0, 0, 1.0),
    facing 90 deg relative to conflict_pt.heading,
    with behavior CrossingBehavior(ego, globalParameters.PED_SPEED, globalParameters.TRIGGER_DIST)

# Hard Requirement: Pedestrian must NOT be on the drivable road.
require ped not in network.drivableRegion

# Terminate when ego moves far enough away
terminate when (distance from ego to spawnPt) > 60