# Scenario: The ego vehicle is turning right; the adversarial car (positioned ahead on the right) blocks the lane by braking suddenly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car travels normally in the ego's target lane just beyond the intersection, then suddenly slams and holds its brakes once the ego closes in, permanently blocking the lane as the ego completes its right turn.

param EGO_SPEED = 7
param ADV_SPEED = 10
param BRAKE_INTENSITY = 1.0
param SAFETY_DIST = 15
param ADV_BRAKE_TRIGGER_DIST = 20

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
    interrupt when (distance from self to ego) < globalParameters.ADV_BRAKE_TRIGGER_DIST:
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego turns right and the adversary is already in the ego's target end lane just past the intersection exit, driving ahead before suddenly braking to block the path.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_trajectory = [ego_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Ego is anchored from the definitive end of its start lane and stepped back; adversary is anchored from the definitive start of the end lane and stepped only a short distance forward, ensuring both vehicles are tightly placed and correctly oriented without any heading misalignment.

param EGO_DIST_TO_INTERSECTION = Range(10, 14)
param ADV_DIST_FROM_INTERSECTION = Range(5, 10)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.end
adv_anchor = new OrientedPoint at ego_maneuver.endLane.centerline.start

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for globalParameters.ADV_DIST_FROM_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100