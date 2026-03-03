# Scenario: The ego vehicle is attempting to change lanes to avoid a slow-moving leading vehicle; the adversarial car in the target lane suddenly slows down, matching the speed of the leading vehicle, effectively blocking the ego from completing the lane change.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The slow vehicle ahead stays in the ego's original lane at low speed, triggering the lane change attempt. The adversarial car initially travels at normal speed in the target lane but suddenly slows to match the slow vehicle's speed once the ego gets close, trapping the ego between two equally slow vehicles.

param EGO_SPEED = 12
param SLOW_SPEED = 4
param ADV_INIT_SPEED = 12
param ADV_MATCH_SPEED = 4
param TRIGGER_DIST = 22
param SAFETY_DIST = 10
param MATCH_TRIGGER_DIST = 18

behavior SlowVehicleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.SLOW_SPEED)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_INIT_SPEED) until \
        (distance to ego) < globalParameters.MATCH_TRIGGER_DIST
    do FollowLaneBehavior(target_speed=globalParameters.ADV_MATCH_SPEED)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to slowVehicle) < globalParameters.TRIGGER_DIST

    rightSection = network.laneSectionAt(self)._laneToRight

    try:
        do LaneChangeBehavior(laneSectionToSwitch=rightSection, target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to adversary) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(0.8)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A multi-lane straight road where the ego and slow vehicle share the left lane, and the adversarial car starts ahead in the right adjacent lane before slowing to block the lane change.

valid_lane_pairs = []
for lane in network.lanes:
    for sec in lane.sections:
        right_sec = sec._laneToRight
        if right_sec is not None:
            valid_lane_pairs.append((lane, right_sec.lane))
            break

selected_pair = Uniform(*valid_lane_pairs)
initLane = selected_pair[0]
rightLane = selected_pair[1]

spawnPt = new OrientedPoint on initLane.centerline
rightPt = new OrientedPoint on rightLane.centerline

require (distance from spawnPt to intersection) > 80
require (distance from rightPt to intersection) > 80

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The slow vehicle is placed ahead of the ego in the same lane. The adversarial car is placed slightly further ahead than the slow vehicle in the right target lane, so that when it slows down it creates a simultaneous blockage in both lanes at the same longitudinal position.

param SLOW_DIST = Range(18, 25)
param ADV_DIST = Range(22, 28)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

slow_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.SLOW_DIST
slowVehicle = new Car at slow_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior SlowVehicleBehavior()

adv_spot = new OrientedPoint following roadDirection from rightPt for globalParameters.ADV_DIST
adversary = new Car at adv_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior AdversaryBehavior()

require (distance from slowVehicle to intersection) > 50
require (distance from adversary to intersection) > 40