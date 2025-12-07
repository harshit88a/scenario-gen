# Scenario: Ego vehicle makes a right turn at a 4-way intersection. An adversary vehicle from the opposite lane makes a left turn. Both vehicles target the same destination lane, creating a collision risk.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Constants
param EGO_SPEED = Range(7, 10)
param ADV_SPEED = Range(10, 13)

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT

behavior ConstantSpeedTrajectory(trajectory, speed):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=speed)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# Helper function to find conflicting pairs in a specific intersection
def get_conflicting_pairs(intersec):
    pairs = []
    # Get all right turns
    right_turns = filter(lambda m: m.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
    # Get all left turns
    left_turns = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
    
    for rt in right_turns:
        for lt in left_turns:
            # Check if they end in the exact same lane
            if rt.endLane == lt.endLane:
                pairs.append([rt, lt])
    return pairs

# A. distinct 4-way intersections
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)

# B. Collect ALL valid pairs from ALL 4-way intersections
all_conflicting_pairs = []
for i in fourWayIntersections:
    local_pairs = get_conflicting_pairs(i)
    for p in local_pairs:
        all_conflicting_pairs.append(p)

# Require that we found at least one pair
require len(all_conflicting_pairs) > 0

# C. Select ONE pair uniformly
selected_pair = Uniform(*all_conflicting_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

# D. Define Lanes and Trajectories
ego_lane = ego_maneuver.startLane
adv_lane = adv_maneuver.startLane

ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param EGO_DIST = Range(10, 15)
param ADV_DIST = Range(15, 20)

# 1. Spawn Ego
ego_init_pt = new OrientedPoint in ego_lane.centerline
ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST

# 2. Spawn Adversary (Opposite side)
adv_init_pt = new OrientedPoint in adv_lane.centerline
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

# 3. Instantiate Agents
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior ConstantSpeedTrajectory(ego_trajectory, globalParameters.EGO_SPEED)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior ConstantSpeedTrajectory(adv_trajectory, globalParameters.ADV_SPEED)

# Requirements
require monitor GreenWave()
require (distance from ego to adversary) > 10
terminate when (distance to ego_spawn_pt) > 70