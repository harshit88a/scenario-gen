# Scenario: Unprotected left turn at intersection where ego yields to an oncoming car while an adversarial car approaching from the right drives extremely slowly and blocks multiple lanes, forcing the ego to change lanes to proceed.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The oncoming car proceeds straight through the intersection at normal speed, requiring the ego to yield. Simultaneously, the adversarial car from the right enters the intersection at an extremely low speed, drifting across multiple lanes and obstructing the ego's intended turning path, forcing a reactive lane change.

param EGO_SPEED = 8
param ONCOMING_SPEED = 13
param ADV_SLOW_SPEED = 2
param SAFETY_DIST = 18
param BRAKE_INTENSITY = 1.0
param LANE_CHANGE_STEER = 0.25
param LANE_CHANGE_TRIGGER_DIST = 20

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(oncoming, 100):
            setClosestTrafficLightStatus(oncoming, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior OncomingBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ONCOMING_SPEED)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SLOW_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        if (distance from self to adversary) < (distance from self to oncoming):
            take SetSteerAction(globalParameters.LANE_CHANGE_STEER), SetThrottleAction(0.5)
        else:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego performs a left turn, the oncoming car proceeds straight from the opposite direction, and the adversarial car enters from the right-side road driving extremely slowly across multiple lanes.

ONCOMING_MODEL = "vehicle.audi.a2"
ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

oncoming_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
oncoming_maneuver = Uniform(*oncoming_maneuvers)
oncoming_trajectory = [oncoming_maneuver.startLane, oncoming_maneuver.connectingLane, oncoming_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, oncoming_maneuver.startLane.maneuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The oncoming car spawns directly opposite the ego at moderate distance. The adversarial slow car spawns on the right-side approach road far enough to be mid-intersection and blocking lanes when the ego begins its turn.
param EGO_DIST_TO_INTERSECTION = Range(12, 18)
param ONCOMING_DIST_TO_INTERSECTION = Range(25, 35)
param ADV_DIST_TO_INTERSECTION = Range(10, 15)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
oncoming_init_pt = new OrientedPoint in oncoming_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION
oncoming_spawn_pt = new OrientedPoint following roadDirection from oncoming_init_pt for -globalParameters.ONCOMING_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

oncoming = new Car at oncoming_spawn_pt offset by (0, 0, 0.5),
    with blueprint ONCOMING_MODEL,
    with behavior OncomingBehavior(oncoming_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100