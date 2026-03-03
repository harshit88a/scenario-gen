# Scenario: Ego drives straight through an intersection; crossing vehicle runs red light and accelerates, forcing ego to perform collision avoidance.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial vehicle runs the red light and suddenly accelerates; the ego detects the conflict and performs emergency braking and steering to avoid collision.

param ADV_INITIAL_SPEED = Range(8, 14)
param ADV_ACCEL_SPEED = Range(22, 32)
param ADV_DIST_TO_INTERSECTION = Range(15, 25)
param EGO_SPEED = 10
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param SAFETY_BRAKE_INTENSITY = 1.0
param AVOIDANCE_STEER = Range(0.3, 0.6)
param THREAT_DISTANCE = 18

behavior RunRedLightAndAccelerateBehavior(initial_speed, accel_speed):
    do FollowLaneBehavior(target_speed=initial_speed) until (distance from self to intersection) < 12
    while True:
        do FollowLaneBehavior(target_speed=accel_speed)

behavior EgoCollisionAvoidanceBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.THREAT_DISTANCE):
        try:
            do FollowLaneBehavior(target_speed=target_speed)
        interrupt when withinDistanceToAnyObjs(self, 10):
            take SetBrakeAction(globalParameters.SAFETY_BRAKE_INTENSITY)
            take SetSteerAction(globalParameters.AVOIDANCE_STEER)


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
### The adversarial car is placed on a conflicting crossing lane and accelerates aggressively after running the red light.

conflicting_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_lane = adv_maneuver.startLane

adv_spawn_pt = new OrientedPoint following roadDirection from adv_lane.centerline[-1] for -globalParameters.ADV_DIST_TO_INTERSECTION

crossing_car = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior RunRedLightAndAccelerateBehavior(
        initial_speed=globalParameters.ADV_INITIAL_SPEED,
        accel_speed=globalParameters.ADV_ACCEL_SPEED
    )

require (distance from ego to crossing_car) < 50