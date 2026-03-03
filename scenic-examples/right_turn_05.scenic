# Scenario: The ego vehicle is turning right; the adversarial vehicle enters the intersection from the left side, swerving to the right suddenly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car approaches from the left and enters the intersection at normal speed, then suddenly swerves right toward the ego's turning path, forcing the ego to brake hard to avoid a collision.

param EGO_SPEED = 7
param ADV_SPEED = 10
param SAFETY_DIST = 15
param BRAKE_INTENSITY = 1.0
param ADV_SWERVE_TRIGGER_DIST = 22
param ADV_STEER_INTENSITY = 0.4

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
    interrupt when (distance from self to ego) < globalParameters.ADV_SWERVE_TRIGGER_DIST:
        while True:
            take SetSteerAction(globalParameters.ADV_STEER_INTENSITY), SetThrottleAction(0.5)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego turns right and the adversary approaches straight from the left-side perpendicular lane, swerving right mid-intersection directly into the ego's turning path.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Ego is anchored from the definitive end of its start lane and stepped back; adversary is anchored from the definitive end of its start lane and stepped back a short distance so it arrives at the intersection just as the ego commits to its right turn.

param EGO_DIST_TO_INTERSECTION = Range(10, 14)
param ADV_DIST_TO_INTERSECTION = Range(14, 20)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.end
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.end

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100