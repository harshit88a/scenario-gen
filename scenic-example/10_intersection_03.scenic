# Scenario: Ego vehicle makes a right turn at a 3-way intersection. An adversary vehicle from a lateral lane goes straight, jumping red signal and causing a conflict.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car jumps a red signal and continues straight through the intersection, creating a collision course with the ego vehicle.

param EGO_SPEED = Range(7, 10)
param ADV_SPEED = Range(8, 12)
param SAFETY_DIST = Range(10, 15)
param BRAKE_INTENSITY = 1.0

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# A three-way (T-junction) intersection.

ADV_MODEL = "vehicle.tesla.model3"

intersections = filter(lambda i: i.is3Way, network.intersections)
intersec = Uniform(*intersections)

ego_maneuvers = filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

conflicting_maneuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is in the lateral lane (the "top" of the T or the perpendicular road), approaching from the left.

param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param ADV_DIST_TO_INTERSECTION = Range(15, 25)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 60