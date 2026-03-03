# Scenario: The ego vehicle performs a lane change to evade a slow-moving vehicle; the adversarial car in the target lane on the right front suddenly brakes, causing the ego vehicle to react quickly to avoid a collision.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The slow vehicle ahead stays in the ego's original lane at low speed, triggering the lane change. The adversarial car starts in the right target lane ahead of the ego and suddenly brakes hard once the ego gets close, creating a collision threat mid lane change.

param EGO_SPEED = 12
param SLOW_SPEED = 4
param ADV_INIT_SPEED = 10
param TRIGGER_DIST = 20
param BRAKE_DIST = 12
param SAFETY_DIST = 8

behavior SlowVehicleBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.SLOW_SPEED)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_INIT_SPEED) until \
        (distance to ego) < globalParameters.BRAKE_DIST
    take SetBrakeAction(1.0)
    wait

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to slowVehicle) < globalParameters.TRIGGER_DIST

    rightSection = network.laneSectionAt(self)._laneToRight

    try:
        do LaneChangeBehavior(laneSectionToSwitch=rightSection, target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to adversary) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(1.0)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A multi-lane straight road where the ego and slow vehicle share the left lane, and the adversarial car is positioned ahead in the right adjacent lane.

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
# The slow vehicle is placed directly ahead of the ego in the same lane. The adversarial car is placed further ahead in the right target lane, moving at moderate speed before braking hard as the ego arrives.

param SLOW_DIST = Range(18, 25)
param ADV_DIST = Range(30, 40)

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