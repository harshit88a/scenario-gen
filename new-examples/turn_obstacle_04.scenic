# Scenario: The ego vehicle is turning left at an intersection; the adversarial cyclist on the left front suddenly stops in the middle of the intersection and dismounts, obstructing the ego vehicle's path.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial cyclist approaches from the left front along the straight conflicting lane, enters the intersection, then abruptly brakes and stops at the midpoint of the ego's left-turn connecting lane, simulating a dismount that directly obstructs the ego's path.

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(4, 7)
param SAFETY_DIST = Range(8, 14)
param BRAKE_INTENSITY = 1.0
param ADV_STOP_THRESHOLD = Range(2, 4)
param EGO_TRIGGER_DIST = Range(10, 15)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(trajectory, stop_point):
    # Wait until ego is close to the intersection before starting
    while (distance from ego to stop_point) > globalParameters.EGO_TRIGGER_DIST:
        take SetThrottleAction(0), SetBrakeAction(0)
    # Cyclist rides into intersection then abruptly stops, simulating dismount
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    interrupt when (distance from self to stop_point) < globalParameters.ADV_STOP_THRESHOLD:
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# Any intersection where the ego performs a left turn with a conflicting straight maneuver from the left front; the cyclist stop point is anchored at the midpoint of the ego's connecting lane to place the obstruction squarely in the turning arc.

ADV_MODEL = "vehicle.bh.crossbike"

def get_conflict_pairs(intersections):
    pairs = []
    for intersec in intersections:
        left_turns = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
        for lt in left_turns:
            conflicts = filter(lambda c: c.type == ManeuverType.STRAIGHT, lt.conflictingManeuvers)
            for c in conflicts:
                pairs.append([lt, c])
    return pairs

all_intersections = network.intersections
valid_pairs = get_conflict_pairs(all_intersections)

require len(valid_pairs) > 0

selected_pair = Uniform(*valid_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

connecting_pts = ego_maneuver.connectingLane.centerline.points
mid_idx = int(len(connecting_pts) / 2)
intersection_stop_pt = new OrientedPoint at connecting_pts[mid_idx]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The ego spawns behind the intersection on its left-turn start lane; the cyclist spawns on the conflicting straight lane at the left front, close enough to the intersection to enter and stop at the connecting lane midpoint as the ego commits to the turn.

param EGO_DIST_TO_INTERSECTION = Range(15, 25)
param ADV_DIST_TO_INTERSECTION = Range(10, 16)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory, intersection_stop_pt)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80