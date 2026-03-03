# Scenario: The ego vehicle attempts to change lanes to avoid a slow-moving leading vehicle; the adversarial car in the target lane suddenly merges into the ego vehicle's original lane, blocking the ego from returning to its initial position.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The leading vehicle moves slowly ahead in the ego's original lane, forcing a lane change. The adversarial vehicle, initially in the target lane, merges into the ego's original lane once the ego begins its lane change, permanently blocking the ego's return path.

param EGO_SPEED = 12
param ADV_SPEED = 10
param LEAD_SPEED = 4
param TRIGGER_DIST = 18
param SAFETY_DIST = 10

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior AdversaryBehavior(origSection):
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance to ego) < globalParameters.TRIGGER_DIST
    do LaneChangeBehavior(laneSectionToSwitch=origSection, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(targetSection):
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to lead) < globalParameters.TRIGGER_DIST

    do LaneChangeBehavior(laneSectionToSwitch=targetSection, target_speed=globalParameters.EGO_SPEED)

    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to adversary) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(0.8)

## 3. GEOMETRY
# A multi-lane straight road where the ego's original lane and an adjacent target lane run in parallel.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 60

checkPt = new OrientedPoint left of spawnPt by 3.5
targetSection = network.laneSectionAt(checkPt)
origSection = network.laneSectionAt(spawnPt)

require targetSection is not None
require origSection is not None
require targetSection.lane != initLane

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The slow lead vehicle is placed ahead of the ego in the same lane. The adversarial car starts in the target lane alongside or slightly ahead of the ego, poised to cut back into the ego's original lane.

param LEAD_DIST = Range(20, 28)
param ADV_DIST = Range(8, 14)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

lead_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DIST
lead = new Car at lead_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

adv_spot = new OrientedPoint following roadDirection from checkPt for globalParameters.ADV_DIST
adversary = new Car at adv_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior AdversaryBehavior(origSection)

require (distance from lead to intersection) > 30
require (distance from adversary to intersection) > 30