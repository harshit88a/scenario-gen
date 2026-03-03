# Scenario: Unprotected left turn at intersection where the ego yields to an oncoming car that suddenly attempts a last-second right turn, creating an unexpected conflict in the ego's intended path.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car approaches the intersection at a normal speed as if going straight, then abruptly executes a last-second right turn, cutting across the ego's intended left-turn path.

param EGO_SPEED = 8
param ADV_SPEED = 14
param SAFETY_DIST = 18
param BRAKE_INTENSITY = 1.0
param ADV_TURN_TRIGGER_DIST = 22

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(straight_trajectory, turn_trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=straight_trajectory, target_speed=globalParameters.ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.ADV_TURN_TRIGGER_DIST:
        do FollowTrajectoryBehavior(trajectory=turn_trajectory, target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego performs a left turn and the adversary initially approaches straight but switches to a right turn at the last second.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_straight_maneuver = Uniform(*adv_straight_maneuvers)
adv_straight_trajectory = [adv_straight_maneuver.startLane, adv_straight_maneuver.connectingLane, adv_straight_maneuver.endLane]

adv_right_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, adv_straight_maneuver.startLane.maneuvers)
adv_right_maneuver = Uniform(*adv_right_maneuvers)
adv_turn_trajectory = [adv_right_maneuver.startLane, adv_right_maneuver.connectingLane, adv_right_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is spawned in the oncoming straight lane at a distance where it appears to be proceeding straight before making its abrupt last-second right turn.
param EGO_DIST_TO_INTERSECTION = Range(12, 18)
param ADV_DIST_TO_INTERSECTION = Range(30, 40)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_straight_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_straight_trajectory, adv_turn_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100