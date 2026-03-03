# Scenario: Ego vehicle makes a right turn at a 4-way intersection. An adversary vehicle from the opposite lane makes a left turn. Both vehicles target the same destination lane, creating a collision risk.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car performs a left turn into the same destination lane as the ego vehicle, failing to yield or creating a simultaneous merge conflict.

param EGO_SPEED = Range(7, 10)
param ADV_SPEED = Range(10, 13)

behavior ConstantSpeedTrajectory(trajectory, speed):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=speed)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# Four-way intersections to find pairs of right and left turns that merge into the same destination lane, setting the structural path for the collision risk.

ADV_MODEL = "vehicle.tesla.model3"

def get_conflicting_pairs(intersec):
    pairs = []
    right_turns = filter(lambda m: m.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
    left_turns = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
    
    for rt in right_turns:
        for lt in left_turns:
            if rt.endLane == lt.endLane:
                pairs.append([rt, lt])
    return pairs

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)

all_conflicting_pairs = []
for i in fourWayIntersections:
    local_pairs = get_conflicting_pairs(i)
    for p in local_pairs:
        all_conflicting_pairs.append(p)

require len(all_conflicting_pairs) > 0

selected_pair = Uniform(*all_conflicting_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

ego_lane = ego_maneuver.startLane
adv_lane = adv_maneuver.startLane

ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Places the ego and adversary in their respective lanes at specific distances from the intersection to facilitate the merging conflict.

param EGO_DIST = Range(10, 15)
param ADV_DIST = Range(15, 20)

ego_init_pt = new OrientedPoint in ego_lane.centerline
ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST

adv_init_pt = new OrientedPoint in adv_lane.centerline
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior ConstantSpeedTrajectory(ego_trajectory, globalParameters.EGO_SPEED)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior ConstantSpeedTrajectory(adv_trajectory, globalParameters.ADV_SPEED)

require monitor GreenWave()
require (distance from ego to adversary) > 10
terminate when (distance to ego_spawn_pt) > 70