# Scenario: Ego vehicle approaches a 4-way intersection for crossing negotiation; the adversarial car on the left suddenly accelerates, enters the intersection first, and abruptly stops, forcing the ego to brake.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car from the left approaches at normal speed, then suddenly accelerates into the intersection ahead of the ego and abruptly stops once inside, blocking the path.

param EGO_SPEED = Range(7, 10)
param ADV_INITIAL_SPEED = Range(4, 6)
param ADV_ACCEL_SPEED = Range(14, 18)
param SAFETY_DIST = Range(10, 15)
param BRAKE_INTENSITY = 1.0
param ADV_TRIGGER_DIST = Range(10, 14)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_INITIAL_SPEED) \
        until (distance from self to intersec) < globalParameters.ADV_TRIGGER_DIST
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_ACCEL_SPEED) \
        until self in intersec
    while True:
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)


## 3. GEOMETRY
# A four-way intersection where the ego travels straight and the adversary approaches from the left on a conflicting straight path.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is spawned in the left-side approach lane, slightly closer to the intersection than the ego so it can plausibly enter first after accelerating.

param EGO_DIST = Range(20, 25)
param ADV_DIST = Range(12, 16)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 70