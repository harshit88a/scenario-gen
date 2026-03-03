# Scenario: Ego vehicle enters a four-way intersection for crossing negotiation; the adversarial vehicle coming from the right makes a left turn and abruptly stops mid-turn, causing a near collision with the ego vehicle.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial vehicle approaches from the right at normal speed, initiates a left turn that crosses directly into the ego's straight path, and then abruptly stops mid-turn inside the intersection, forcing the ego into a near-collision situation.

param EGO_SPEED = 9
param ADV_SPEED = 10
param SAFETY_DIST = 15
param BRAKE_INTENSITY = 1.0
param ADV_BRAKE_INTENSITY = 1.0
param ADV_STOP_TRIGGER_DIST = 22

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(trajectory, connectingLane):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.ADV_STOP_TRIGGER_DIST:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED) until (self in connectingLane)
        while True:
            take SetBrakeAction(globalParameters.ADV_BRAKE_INTENSITY)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego proceeds straight and the adversary approaches from the right side road and performs a conflicting left turn, swinging directly across the ego's straight path and stopping mid-turn.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial vehicle is spawned on the right-side approach lane at a moderate distance; a require constraint enforces that it is geometrically to the right of the ego at spawn, guaranteeing the right-side approach condition. Its left turn swings it directly into the ego's path where it stops mid-turn.

param EGO_DIST_TO_INTERSECTION = Range(12, 18)
param ADV_DIST_TO_INTERSECTION = Range(20, 30)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory, adv_maneuver.connectingLane)

require (adversary.position relative to ego_spawn_pt).x > 0
require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100