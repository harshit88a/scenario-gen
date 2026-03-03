# Scenario: The ego vehicle is turning right at an intersection; the adversarial pedestrian on the left front suddenly crosses the road and stops in the middle of the intersection, blocking the ego vehicle's path.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial pedestrian waits idle until the ego has entered the right-turn connecting lane, then suddenly crosses the ego's turning arc and freezes at the centerline, blocking the ego mid-turn.

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(1.2, 2.0)
param SAFETY_DIST = Range(8, 14)
param BRAKE_INTENSITY = 1.0
param ADV_STOP_THRESHOLD = 2.0
param ADV_LATERAL_OFFSET = Range(4, 6)
param EGO_TRIGGER_DIST = Range(8, 12)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(stop_point):
    # Wait idle until ego has entered the right-turn arc
    while (distance from ego to stop_point) > globalParameters.EGO_TRIGGER_DIST:
        take SetWalkingSpeedAction(0)
    # Ego has turned right — pedestrian suddenly crosses and stops in the middle
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
# Any intersection where the ego performs a right turn; the pedestrian is placed orthogonally to the ego's connecting lane so the crossing path intersects the turning arc directly.

ADV_MODEL = "walker.pedestrian.0001"

def get_right_turn_maneuvers(intersections):
    maneuvers = []
    for intersec in intersections:
        right_turns = filter(lambda m: m.type == ManeuverType.RIGHT_TURN, intersec.maneuvers)
        for rt in right_turns:
            maneuvers.append(rt)
    return maneuvers

all_intersections = network.intersections
valid_maneuvers = get_right_turn_maneuvers(all_intersections)

require len(valid_maneuvers) > 0

ego_maneuver = Uniform(*valid_maneuvers)
ego_trajectory = [ego_maneuver.startLane, ego_maneuver.connectingLane, ego_maneuver.endLane]

intersection_stop_pt = new OrientedPoint at ego_maneuver.connectingLane.centerline.points[0]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The ego spawns behind the intersection on its right-turn start lane; the pedestrian spawns laterally offset to the left of the connecting lane entry, stands idle until the ego completes the turn, then crosses and freezes to block the ego's exit path.

param EGO_DIST_TO_INTERSECTION = Range(15, 25)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION

adv_spawn_pt = new OrientedPoint at intersection_stop_pt offset by (globalParameters.ADV_LATERAL_OFFSET, 0, 0)

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Pedestrian at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with regionContainedIn None,
    facing toward intersection_stop_pt,
    with behavior AdversaryBehavior(intersection_stop_pt)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80