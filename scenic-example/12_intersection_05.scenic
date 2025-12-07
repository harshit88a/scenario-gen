# Scenario: Ego vehicle goes straight at a 3-way intersection. An adversary vehicle makes a left turn from a conflicting lane. The Ego must brake suddenly to avoid a collision.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Constants
param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(10, 14)
param SAFETY_DIST = Range(8, 12)
param BRAKE_INTENSITY = 1.0

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# Helper to find valid Straight vs Left Turn pairs at 3-way intersections
def get_conflict_pairs(intersections):
    pairs = []
    for intersec in intersections:
        # Get all Straight maneuvers (Potential Ego)
        straights = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers)
        
        for s in straights:
            # Find Left Turns that conflict with this Straight maneuver
            # We look inside s.conflictingManeuvers for Left Turns
            conflicts = filter(lambda c: c.type == ManeuverType.LEFT_TURN, s.conflictingManeuvers)
            
            for c in conflicts:
                pairs.append([s, c])
    return pairs

# A. Find all 3-way intersections
three_way_intersections = filter(lambda i: i.is3Way, network.intersections)

# B. Pre-calculate valid maneuver pairs to avoid RandomControlFlowError
valid_pairs = get_conflict_pairs(three_way_intersections)

# Requirement: Ensure we found at least one valid configuration in the map
require len(valid_pairs) > 0

# C. Select one pair Uniformly
selected_pair = Uniform(*valid_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

# D. Define Trajectories
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param EGO_DIST_TO_INTERSECTION = Range(15, 25)
param ADV_DIST_TO_INTERSECTION = Range(15, 20)

# 1. Define Anchor Points (The intersection entry points)
# Using .points[-1] of the start lane centerline gives us the stop line/entry point
ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

# 2. Define Spawn Points relative to the Anchors (Negative distance = upstream)
ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

# 3. Instantiate Agents
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

# Requirements and Termination
require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80