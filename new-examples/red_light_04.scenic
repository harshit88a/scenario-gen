# Scenario: Ego moves straight through an intersection; a crossing vehicle runs the red light from the right and brakes abruptly, forcing the ego to perform a collision avoidance maneuver.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial vehicle runs the red light from the right, enters the intersection connecting lane, then brakes abruptly; the ego detects the obstruction and performs emergency braking to avoid collision.

param ADV_SPEED = Range(10, 18)
param ADV_DIST_TO_INTERSECTION = Range(15, 25)
param EGO_SPEED = 10
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param SAFETY_BRAKE_INTENSITY = 1.0
param ADV_BRAKE_INTENSITY = 1.0
param THREAT_DISTANCE = 15

behavior RunRedLightAndBrakeBehavior(target_speed, maneuver):
    # Phase 1: approach the intersection at speed until entering the connecting lane
    do FollowLaneBehavior(target_speed=target_speed) until (self in maneuver.connectingLane)
    # Phase 2: abruptly brake once inside the intersection
    while True:
        take SetBrakeAction(globalParameters.ADV_BRAKE_INTENSITY)

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
### The adversarial car is placed on a straight conflicting lane approaching from the right, runs the red light, and brakes abruptly upon entering the intersection to obstruct the ego.

conflicting_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_lane = adv_maneuver.startLane

adv_spawn_pt = new OrientedPoint following roadDirection from adv_lane.centerline[-1] for -globalParameters.ADV_DIST_TO_INTERSECTION

crossing_car = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior RunRedLightAndBrakeBehavior(target_speed=globalParameters.ADV_SPEED, maneuver=adv_maneuver)

require (distance from ego to crossing_car) < 50