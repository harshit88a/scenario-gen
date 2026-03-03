# Scenario: Ego vehicle goes straight through an intersection; the adversarial vehicle approaches from the left front and cuts off the ego vehicle.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial vehicle approaches from the left front, accelerates through the intersection to cut off the ego's path, then continues; the ego detects the cut-off and performs emergency braking to avoid collision.

param ADV_APPROACH_SPEED = Range(8, 14)
param ADV_CUTOFF_SPEED = Range(18, 26)
param ADV_DIST_TO_INTERSECTION = Range(15, 25)
param EGO_SPEED = 10
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param SAFETY_BRAKE_INTENSITY = 1.0
param THREAT_DISTANCE = 15

behavior CutOffBehavior(approach_speed, cutoff_speed, maneuver):
    # Phase 1: approach the intersection at normal speed
    do FollowLaneBehavior(target_speed=approach_speed) until (self in maneuver.connectingLane)
    # Phase 2: accelerate aggressively through the intersection to cut off ego
    do FollowLaneBehavior(target_speed=cutoff_speed, laneToFollow=maneuver.connectingLane)
    # Phase 3: continue on exit lane at cutoff speed
    while True:
        do FollowLaneBehavior(target_speed=cutoff_speed)

behavior EgoCollisionAvoidanceBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.THREAT_DISTANCE):
        take SetBrakeAction(globalParameters.SAFETY_BRAKE_INTENSITY)


## 3. GEOMETRY
### Defining a 4-way intersection and the ego vehicle's straight path through it.

ADV_MODEL = "vehicle.tesla.model3"

intersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*intersections)

startLane = Uniform(*intersec.incomingLanes)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)

ego_spawn_pt = new OrientedPoint following roadDirection from startLane.centerline[-1] for -globalParameters.EGO_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoCollisionAvoidanceBehavior(target_speed=globalParameters.EGO_SPEED)


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The adversarial car is placed on a conflicting left-turn lane from the left front, accelerating through the intersection to cut directly across the ego's path.

conflicting_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_lane = adv_maneuver.startLane

adv_spawn_pt = new OrientedPoint following roadDirection from adv_lane.centerline[-1] for -globalParameters.ADV_DIST_TO_INTERSECTION

crossing_car = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior CutOffBehavior(
        approach_speed=globalParameters.ADV_APPROACH_SPEED,
        cutoff_speed=globalParameters.ADV_CUTOFF_SPEED,
        maneuver=adv_maneuver
    )

require (distance from ego to crossing_car) < 50