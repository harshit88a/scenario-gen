# Scenario: Ego goes straight at an intersection; Crossing vehicle runs a red light.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Parameters for the Adversary
param ADV_SPEED = Range(10, 20)  # Fast enough to represent a dangerous violation
param ADV_DIST_TO_INTERSECTION = Range(15, 25)

# Parameters for the Ego
param EGO_SPEED = 10
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param SAFETY_BRAKE_INTENSITY = 1.0

# Behavior: The adversary ignores traffic logic and maintains speed through the intersection
behavior RunRedLightBehavior(target_speed):
    while True:
        # We explicitly set target speed. In Scenic 3.0 + CARLA, high target speed
        # combined with right-of-way conflicts usually forces the agent to proceed.
        # Ideally, we rely on the solver to pick a timing where the light is Red for Adv,
        # but behaviors here force the movement regardless of the light state.
        do FollowLaneBehavior(target_speed=target_speed)

# Behavior: Ego drives but brakes if a collision is imminent (Collision Avoidance)
behavior EgoSafetyBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, 10):
        take SetBrakeAction(globalParameters.SAFETY_BRAKE_INTENSITY)

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# 3.1 Find a 4-Way Intersection
intersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*intersections)

# 3.2 Select Ego Lane and Maneuver (Going Straight)
# We pick a random incoming lane to the intersection
startLane = Uniform(*intersec.incomingLanes)

# Filter for a STRAIGHT maneuver from this lane
ego_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)

# 3.3 Create Ego Spawn Point
# We place the ego "backwards" from the intersection by the parameter distance
ego_spawn_pt = new OrientedPoint following roadDirection from startLane.centerline[-1] for -globalParameters.EGO_DIST_TO_INTERSECTION

# 3.4 Spawn Ego
ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoSafetyBehavior(target_speed=globalParameters.EGO_SPEED)

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT

# 4.1 Identify the Crossing Lane
# We look for a maneuver that conflicts with the Ego's straight path.
# We specifically filter for another STRAIGHT maneuver (which implies a crossing path in a 4-way)
conflicting_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, ego_maneuver.conflictingManeuvers)
adv_maneuver = Uniform(*conflicting_maneuvers)
adv_lane = adv_maneuver.startLane

# 4.2 Create Adversary Spawn Point
# We place the adversary "backwards" from its entry point to the intersection
adv_spawn_pt = new OrientedPoint following roadDirection from adv_lane.centerline[-1] for -globalParameters.ADV_DIST_TO_INTERSECTION

# 4.3 Spawn Adversary
# The adversary is spawned on the crossing path with the Red Light Running behavior
crossing_car = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior RunRedLightBehavior(target_speed=globalParameters.ADV_SPEED)

# Requirement: Ensure the vehicles are actually close enough to interact
require (distance from ego to crossing_car) < 50