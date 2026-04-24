# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = Range(7, 10)
param ADV_REVERSE_SPEED = -5
param TRIGGER_DIST = Range(10, 15)
param SAFETY_DIST = Range(10, 15)
param BRAKE_INTENSITY = 1.0

behavior AdversaryBehavior(target_ego):
    do FollowLaneBehavior(target_speed=0) until \
        (distance from self to target_ego) < globalParameters.TRIGGER_DIST
    do FollowLaneBehavior(target_speed=globalParameters.ADV_REVERSE_SPEED)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

# 3. ROAD GEOMETRY
ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

ego_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
ego_maneuver = Uniform(*ego_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param EGO_DIST_TO_INTERSECTION = Range(15, 20)
param ADV_DIST_FROM_INTERSECTION = Range(10, 20)

ego_init_pt = new OrientedPoint in ego_maneuver.startLane.centerline
ego_spawn_pt = new OrientedPoint following roadDirection from ego_init_pt for -globalParameters.EGO_DIST_TO_INTERSECTION

exit_lane_start = ego_maneuver.endLane.centerline[0]
exit_point = new OrientedPoint at exit_lane_start
adv_spawn_pt = new OrientedPoint following roadDirection from exit_point for globalParameters.ADV_DIST_FROM_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(ego)

terminate when (distance to ego_spawn_pt) > 100