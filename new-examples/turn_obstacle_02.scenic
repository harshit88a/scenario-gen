# Scenario: The ego vehicle is turning left at an intersection; the adversarial pedestrian on the opposite sidewalk suddenly crosses the road from the right front and stops in the middle of the intersection.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian starts on the opposing right-front lane entry, walks briskly into the intersection, and freezes in the middle, forcing the ego to react mid left-turn.

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(1.2, 2.0)
param SAFETY_DIST = Range(8, 14)
param BRAKE_INTENSITY = 1.0
param ADV_STOP_THRESHOLD = 2.0

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(stop_point):
    try:
        while True:
            take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(globalParameters.ADV_SPEED)
    interrupt when (distance from self to stop_point) < globalParameters.ADV_STOP_THRESHOLD:
        while True:
            take SetWalkingSpeedAction(0)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# Any intersection where the ego performs a left turn with a conflicting straight maneuver; both 3-way and 4-way intersections are considered to maximize valid pair availability in Town05.

ADV_MODEL = "walker.pedestrian.0001"

def get_conflict_pairs(intersections):
    pairs = []
    for intersec in intersections:
        left_turns = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersec.maneuvers)
        for lt in left_turns:
            conflicts = filter(lambda c: c.type == ManeuverType.STRAIGHT, lt.conflictingManeuvers)
            for c in conflicts:
                pairs.append([lt, c])
    return pairs

all_intersections = network.intersections
valid_pairs = get_conflict_pairs(all_intersections)

require len(valid_pairs) > 0

selected_pair = Uniform(*valid_pairs)
ego_maneuver = selected_pair[0]
adv_maneuver = selected_pair[1]

ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

intersection_stop_pt = new OrientedPoint at ego_maneuver.connectingLane.centerline.points[0]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The ego spawns on its left-turn start lane; the pedestrian spawns at the entry of the conflicting straight lane on the right front, oriented toward the intersection center where it will freeze mid-crossing.

param EGO_DIST_TO_INTERSECTION = Range(15, 25)
param ADV_DIST_TO_INTERSECTION = Range(6, 10)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
adv_anchor = new OrientedPoint at adv_maneuver.startLane.centerline.points[-1]

ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION
adv_spawn_pt = new OrientedPoint following roadDirection from adv_anchor for -globalParameters.ADV_DIST_TO_INTERSECTION

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Pedestrian at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior(intersection_stop_pt)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80