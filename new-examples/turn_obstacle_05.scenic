# Scenario: The ego vehicle is turning right at an intersection; the adversarial motorcyclist on the opposite sidewalk abruptly crosses the road from the right front and comes to a halt in the center of the intersection.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial motorcyclist waits idle on the opposite sidewalk until the ego has committed to the right turn, then abruptly accelerates across the ego's turning arc and brakes to a halt at the midpoint of the connecting lane, blocking the ego's path.

param EGO_SPEED = Range(6, 10)
param ADV_SPEED = Range(8, 12)
param SAFETY_DIST = Range(8, 14)
param BRAKE_INTENSITY = 1.0
param ADV_STOP_THRESHOLD = Range(2, 4)
param EGO_TRIGGER_DIST = Range(8, 12)
param ADV_LATERAL_OFFSET = Range(5, 8)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

behavior AdversaryBehavior(stop_point):
    # Wait idle until ego has committed to the right turn
    while (distance from ego to stop_point) > globalParameters.EGO_TRIGGER_DIST:
        take SetThrottleAction(0), SetBrakeAction(0)
    # Motorcyclist abruptly crosses and halts at the intersection center
    try:
        while True:
            take SetThrottleAction(1.0), SetSteerAction(0.0)
    interrupt when (distance from self to stop_point) < globalParameters.ADV_STOP_THRESHOLD:
        while True:
            take SetBrakeAction(globalParameters.BRAKE_INTENSITY)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# Any intersection where the ego performs a right turn; the motorcyclist is placed laterally offset to the right of the ego's connecting lane midpoint, facing directly across the turning arc, so the halt occurs squarely in the center of the intersection.

ADV_MODEL = "vehicle.harley-davidson.low_rider"

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

connecting_pts = ego_maneuver.connectingLane.centerline.points
mid_idx = int(len(connecting_pts) / 2)
intersection_stop_pt = new OrientedPoint at connecting_pts[mid_idx]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The ego spawns behind the intersection on its right-turn start lane; the motorcyclist spawns laterally offset to the right of the connecting lane midpoint on the opposite sidewalk, facing toward the stop point, so the crossing path cuts directly through the ego's right-turn arc.

param EGO_DIST_TO_INTERSECTION = Range(15, 25)

ego_anchor = new OrientedPoint at ego_maneuver.startLane.centerline.points[-1]
ego_spawn_pt = new OrientedPoint following roadDirection from ego_anchor for -globalParameters.EGO_DIST_TO_INTERSECTION

adv_spawn_pt = new OrientedPoint at intersection_stop_pt offset by (globalParameters.ADV_LATERAL_OFFSET, 0, 0)

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_trajectory)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with regionContainedIn None,
    facing toward intersection_stop_pt,
    with behavior AdversaryBehavior(intersection_stop_pt)

require monitor GreenWave()
terminate when (distance to ego_spawn_pt) > 80