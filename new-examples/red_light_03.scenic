# Scenario: Ego drives straight through an intersection; adversarial vehicle on the left front runs the red light and makes an abrupt left turn, forcing the ego to perform a collision avoidance maneuver.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial vehicle ignores the red light, explicitly follows its left-turn maneuver through the intersection connecting lane, then continues; the ego detects the intrusion and performs emergency braking to avoid collision.

param ADV_SPEED = Range(8, 16)
param ADV_DIST_TO_INTERSECTION = Range(15, 25)
param EGO_SPEED = 10
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param SAFETY_BRAKE_INTENSITY = 1.0
param THREAT_DISTANCE = 15

behavior RunRedLightAndTurnLeftBehavior(target_speed, maneuver):
    # Phase 1: approach the intersection on the start lane
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to maneuver.connectingLane.centerline[0]) < 8
    # Phase 2: follow the connecting (turning) lane through the intersection
    do FollowLaneBehavior(target_speed=target_speed, laneToFollow=maneuver.connectingLane)
    # Phase 3: continue on the exit lane
    while True:
        do FollowLaneBehavior(target_speed=target_speed)

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
### The adversarial car is placed on a conflicting lane whose maneuver is a left turn; it is given its maneuver explicitly so it physically executes the turn through the intersection.

conflicting_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_lane = adv_maneuver.startLane

adv_spawn_pt = new OrientedPoint following roadDirection from adv_lane.centerline[-1] for -globalParameters.ADV_DIST_TO_INTERSECTION

crossing_car = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior RunRedLightAndTurnLeftBehavior(target_speed=globalParameters.ADV_SPEED, maneuver=adv_maneuver)

require (distance from ego to crossing_car) < 50