# Scenario: Ego vehicle enters a four-way intersection for crossing negotiation; the adversarial car from the right drives extremely slowly, blocking multiple lanes and forcing the ego vehicle to perform a lane change to proceed.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"



## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car enters from the right and crawls through the intersection at an extremely low speed along its trajectory, occupying the ego's path and forcing a lane change.

param EGO_SPEED = 9
param ADV_SPEED = 2
param SAFETY_DIST = 15
param BRAKE_INTENSITY = 1.0
param LANE_CHANGE_TRIGGER_DIST = 25

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance from self to adversary) < globalParameters.LANE_CHANGE_TRIGGER_DIST and (adversary.speed < 3):
        adjacentLane = self.laneSection.laneToLeft
        do LaneChangeBehavior(laneSectionToSwitch=adjacentLane, is_oppositeTraffic=False, target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A four-way intersection where the ego proceeds straight and the adversary enters from the right also going straight at a crawl, occupying the ego's intended path and requiring the ego to shift to an adjacent lane to pass.

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
# The adversarial car is spawned on the right-side conflicting lane at a close distance so its slow crawl immediately enters and blocks the intersection as the ego arrives, with a geometric require ensuring it originates from the right.

param EGO_DIST_TO_INTERSECTION = Range(12, 18)
param ADV_DIST_TO_INTERSECTION = Range(10, 16)

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

require (adversary.position relative to ego_spawn_pt).x > 0
require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100