# Scenario: The ego encounters a parked car blocking its lane and must use the opposite lane to bypass the vehicle when an oncoming car suddenly accelerates, closing the gap for the ego to safely return to its lane, necessitating the ego to quickly decide whether to accelerate or brake to avoid a collision.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The parked car remains stationary in the ego's lane, forcing the ego to cross into the opposite lane. The oncoming car initially travels at a moderate speed toward the ego but suddenly accelerates once the ego has committed to the opposite lane, rapidly closing the gap and forcing the ego to make an emergency decision to either brake hard and abort the bypass or accelerate to complete the maneuver and return to its lane before a head-on collision.

param EGO_SPEED = 10
param ONCOMING_INIT_SPEED = 7
param ONCOMING_ACCEL_SPEED = 18
param BYPASS_TRIGGER_DIST = 20
param ACCEL_TRIGGER_DIST = 30
param SAFETY_DIST = 12
param BRAKE_THRESHOLD = 8

behavior ParkedCarBehavior():
    take SetSpeedAction(0)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_INIT_SPEED) until \
        (distance to ego) < globalParameters.ACCEL_TRIGGER_DIST
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_ACCEL_SPEED)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to parkedCar) < globalParameters.BYPASS_TRIGGER_DIST

    leftSection = network.laneSectionAt(self)._laneToLeft

    try:
        do LaneChangeBehavior(laneSectionToSwitch=leftSection, target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to oncomingCar) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(0.9)

    try:
        do LaneChangeBehavior(laneSectionToSwitch=network.laneSectionAt(self)._laneToRight,
                              target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to oncomingCar) < globalParameters.BRAKE_THRESHOLD:
        take SetBrakeAction(1.0)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A two-way road where the ego and parked car share one lane direction, and the oncoming car travels in the adjacent opposite-direction lane. The ego must temporarily occupy the oncoming lane to pass the parked car, creating a direct conflict with the accelerating oncoming vehicle.

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
# The parked car is placed directly ahead of the ego in the same lane, simulating a lane blockage. The oncoming car is placed further ahead in the opposing lane, initially appearing to leave enough room for the ego to bypass and return, but then accelerating to collapse that margin and force an emergency response from the ego.

param PARKED_DIST = Range(18, 26)
param ONCOMING_DIST = Range(35, 50)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

parked_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.PARKED_DIST
parkedCar = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior ParkedCarBehavior()

oncoming_spot = new OrientedPoint following roadDirection from oncomingPt for globalParameters.ONCOMING_DIST
oncomingCar = new Car at oncoming_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior OncomingBehavior()

require (distance from parkedCar to intersection) > 60
require (distance from oncomingCar to intersection) > 40