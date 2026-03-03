# Scenario: The ego vehicle is turning left at an intersection; the adversarial motorcyclist on the right front pretends to cross the road but brakes abruptly at the edge of the road, causing confusion.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial motorcyclist rides forward from its spawn point toward the intersection edge at normal speed, then abruptly brakes and holds position, simulating a false crossing attempt that confuses the ego mid left-turn.

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(8, 12)
param SAFETY_DIST = Range(8, 14)
param BRAKE_INTENSITY = 1.0
param ADV_BRAKE_TRIGGER_DIST = Range(4, 7)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(trajectory, brake_point):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    interrupt when (distance from self to brake_point) < globalParameters.ADV_BRAKE_TRIGGER_DIST:
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# A 4-way intersection where ego performs a left turn; conflicting maneuver pairs are selected such that the adversary's straight-crossing lane originates from the ego's right front, maximizing the false-start confusion effect.

ADV_MODEL = "vehicle.harley-davidson.low_rider"

def get_conflict_pairs(intersections):
    pairs = []
    for intersec in intersections:
        left_turns = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
        for lt in left_turns:
            conflicts = filter(lambda c: c.type == ManeuverType.STRAIGHT, lt.conflictingManeuvers)
            for c in conflicts:
                pairs.append([lt, c])
    return pairs

four_way_intersections = filter(lambda i: i.is4Way, network.intersections)
valid_pairs = get_conflict_pairs(four_way_intersections)

require len(valid_pairs) > 0

selected_pair = Uniform(*valid_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial motorcyclist spawns on the right-front lane relative to the ego; the brake_point is anchored at the intersection entry edge of the adversary's start lane so the abrupt stop occurs visibly at the road boundary.

param EGO_DIST_TO_INTERSECTION = Range(15, 25)
param ADV_DIST_TO_INTERSECTION = Range(12, 18)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

adv_brake_point = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Motorcycle at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory, adv_brake_point)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80