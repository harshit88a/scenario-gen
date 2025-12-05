# Scenario: Ego vehicle goes straight at 4-way intersection and must suddenly stop to 
# avoid collision when adversary vehicle from opposite lane makes a left turn.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Constants
param EGO_SPEED = Range(7, 10)
param ADV_SPEED = Range(7, 10)
param SAFETY_DIST = Range(10, 15)
param BRAKE_INTENSITY = 1.0

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Monitor to freeze traffic lights to Green so the interaction happens
# without vehicles stopping for red lights.
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

# A. Find a 4-way intersection
# We need a 4-way intersection to ensure there is a valid opposite lane.
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

# B. Select Ego Maneuver (Straight)
# Pick a straight maneuver through this intersection.
ego_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

# C. Select Adversary Maneuver (Left Turn from Opposite)
# We find maneuvers that conflict with the Ego.
# Specifically, we want a LEFT_TURN that cuts across the Ego's straight path.
adv_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param EGO_DIST = Range(20, 25)
param ADV_DIST = Range(15, 20)

# Create anchor points at the start of the maneuvers (intersection entry)
ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
adv_init_pt = new OrientedPoint in adv_maneuver.startLane.centerline

# Spawn vehicles upstream (backwards) from the intersection
# Note: We use -distance with positive roadDirection to move backwards safely
ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

# Instantiate Agents
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

# Requirements
require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 70