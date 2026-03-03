# Scenario: Ego vehicle approaches a four-way intersection for crossing negotiation; the adversarial car from the right suddenly accelerates, cuts into the intersection first, clears it, and then abruptly stops just past the intersection, disrupting the ego's path negotiation.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car on the right initially crawls toward the intersection, then suddenly surges to enter and cross the intersection ahead of the ego, and abruptly brakes to a full stop only after clearing the intersection onto the exit lane — continuously holding the brake.

param EGO_SPEED = 8
param ADV_SPEED = 7
param ADV_FAST_SPEED = 22
param SAFETY_DIST = 15
param BRAKE_INTENSITY = 1.0
param ADV_BRAKE_INTENSITY = 1.0
param ADV_ACCEL_TRIGGER_DIST = 30

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(trajectory, endLane):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)
    interrupt when (distance from self to ego) < globalParameters.ADV_ACCEL_TRIGGER_DIST:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_FAST_SPEED) until (self in endLane)
        while True:
            take SetBrakeAction(globalParameters.ADV_BRAKE_INTENSITY)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego proceeds straight and the adversary approaches from the right on a conflicting straight path, creating a right-of-way contest.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is spawned in the right-side conflicting lane at a close-to-moderate distance so that its sudden acceleration plausibly lets it reach and clear the intersection just before the ego arrives, then holds a full stop on the exit lane.

param EGO_DIST_TO_INTERSECTION = Range(14, 20)
param ADV_DIST_TO_INTERSECTION = Range(18, 26)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory, adv_maneuver.endLane)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100