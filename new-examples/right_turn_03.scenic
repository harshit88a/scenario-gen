# Scenario: The ego vehicle is turning right; the adversarial car (positioned ahead on the right) reverses abruptly.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car moves forward briefly in the ego's target lane, then abruptly shifts into reverse and backs toward the intersection exit, directly into the path of the ego completing its right turn.

param EGO_SPEED = 12
param ADV_SPEED = 6
param ADV_REVERSE_SPEED = 8
param SAFETY_DIST = 10
param BRAKE_INTENSITY = 1.0
param ADV_REVERSE_TRIGGER_DIST = 15

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
    interrupt when (distance from self to ego) < globalParameters.ADV_REVERSE_TRIGGER_DIST:
        take SetReverseAction(True)
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_REVERSE_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. GEOMETRY
# A four-way intersection where the ego turns right and the adversary is already positioned just ahead in the ego's end lane, moving forward initially before reversing back into the ego's path.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

adv_trajectory = [ego_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Ego is anchored from the definitive end of its start lane and stepped back; adversary is anchored from the definitive start of the end lane and placed only a short distance forward, ensuring both are tightly positioned and correctly oriented so the reversal immediately threatens the ego mid-turn.

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