# Scenario: Unprotected left turn at intersection with oncoming traffic. The ego-vehicle is performing an unprotected left turn at an intersection, yielding to oncoming traffic.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Constants
param EGO_SPEED = 10
param ADV_SPEED = 15
param SAFETY_DIST = 20
param BRAKE_INTENSITY = 1.0

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Monitor to force green lights so the unprotected turn logic (yield behavior) is required
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
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# Find a 4-way intersection to ensure valid oncoming traffic lanes
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

# Define Ego Maneuver (Left Turn)
ego_maneuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

# Define Adversary Maneuver (Straight, conflicting with Ego)
# We select from maneuvers that conflict with the ego's left turn
adv_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param ADV_DIST_TO_INTERSECTION = Range(20, 25)

# Create anchor points in the center of the respective lanes
ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

# Spawn Ego upstream (backwards) from the anchor point
ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION

# Spawn Adversary upstream
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST_TO_INTERSECTION

# Instantiate Agents
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

# Requirements
require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 100