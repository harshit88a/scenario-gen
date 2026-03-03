# Scenario: The ego encounters a parked car blocking its lane and must use the opposite lane to bypass the vehicle. An oncoming car traveling in the opposite direction (head-on) maintains its speed without yielding, closing the gap and forcing the ego to urgently decide whether to accelerate through the bypass or brake and abort to avoid a head-on collision.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The parked car remains completely stationary in the ego's lane, forcing the ego to cross into the opposing lane to bypass it. The oncoming car travels in the true opposite direction (head-on toward the ego) at a constant threatening speed without decelerating or yielding, meaning the window for the ego to safely complete the bypass and return to its lane continuously shrinks. The ego must decide in real time whether to accelerate and complete the maneuver or brake hard and pull back before a head-on impact occurs.

param EGO_SPEED = 10
param ONCOMING_SPEED = 12
param BYPASS_TRIGGER_DIST = 20
param SAFETY_DIST = 15
param BRAKE_THRESHOLD = 8

behavior ParkedCarBehavior():
    take SetSpeedAction(0)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to parkedCar) < globalParameters.BYPASS_TRIGGER_DIST

    leftSection = network.laneSectionAt(self)._laneToLeft

    try:
        do LaneChangeBehavior(laneSectionToSwitch=leftSection,
                              target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to oncomingCar) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(0.9)

    try:
        do LaneChangeBehavior(laneSectionToSwitch=network.laneSectionAt(self)._laneToRight,
                              target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to oncomingCar) < globalParameters.BRAKE_THRESHOLD:
        take SetBrakeAction(1.0)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A two-way road where the ego and parked car occupy one directional lane, and the oncoming car occupies the opposing directional lane facing directly toward the ego. The oncoming car's lane naturally runs antiparallel to the ego's lane, ensuring a true head-on conflict when the ego crosses into that lane to bypass the parked car.

valid_lane_pairs = []
for lane in network.lanes:
    for sec in lane.sections:
        left_sec = sec._laneToLeft
        if left_sec is not None:
            valid_lane_pairs.append((lane, left_sec.lane))
            break

selected_pair = Uniform(*valid_lane_pairs)
egoLane = selected_pair[0]
oncomingLane = selected_pair[1]

spawnPt = new OrientedPoint on egoLane.centerline
oncomingPt = new OrientedPoint on oncomingLane.centerline

require (distance from spawnPt to intersection) > 90
require (distance from oncomingPt to intersection) > 90

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The parked car is placed directly ahead of the ego in the same lane to create the blockage. The oncoming car is placed ahead in the opposing lane, oriented in the reverse road direction so that it travels head-on toward the ego's bypass path. Its proximity is tuned so that it is visible and threatening from the moment the ego initiates the lane change, leaving a tight but ambiguous time window that forces the ego into a genuine collision avoidance decision.

param PARKED_DIST = Range(18, 26)
param ONCOMING_DIST = Range(30, 45)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

parked_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.PARKED_DIST
parkedCar = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior ParkedCarBehavior()

oncoming_spot = new OrientedPoint following roadDirection from oncomingPt for globalParameters.ONCOMING_DIST
oncomingCar = new Car at oncoming_spot offset by (0, 0, 0.5),
    facing roadDirection offset by 180 deg,
    with blueprint EGO_MODEL,
    with behavior OncomingBehavior()

require (distance from parkedCar to intersection) > 60
require (distance from oncomingCar to intersection) > 40