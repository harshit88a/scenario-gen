# Scenario: Unprotected left turn at intersection where ego yields to an oncoming car whose throttle malfunctions, causing sudden unexpected acceleration and forcing the ego to urgently modify its turning path.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial oncoming car experiences a throttle malfunction mid-approach, causing it to suddenly accelerate beyond its nominal speed and close the gap with the ego vehicle faster than expected.

param EGO_SPEED = 8
param ADV_SPEED = 12
param ADV_SURGE_SPEED = 28
param SAFETY_DIST = 18
param BRAKE_INTENSITY = 1.0
param THROTTLE_FAULT_DIST = 30

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.THROTTLE_FAULT_DIST:
        take SetThrottleAction(1.0)
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SURGE_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego is turning left and the adversary is approaching straight from the opposing direction.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is spawned in the oncoming straight lane, positioned close enough to the intersection to trigger its throttle malfunction while the ego is mid-turn.
param EGO_DIST_TO_INTERSECTION = Range(12, 18)
param ADV_DIST_TO_INTERSECTION = Range(35, 45)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100