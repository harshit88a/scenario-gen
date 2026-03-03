# Scenario: Ego vehicle performs a lane change to bypass a slow adversary vehicle but cannot return to its original lane because the adversary accelerates. Ego vehicle must then slow down to avoid collision with leading vehicle in new lane.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car (slow vehicle) accelerates as the ego attempts to merge back, intentionally blocking the lane change and forcing the ego to interact with a secondary leading vehicle.

param EGO_SPEED = 12
param ADV_INIT_SPEED = 5
param ADV_END_SPEED = 15     
param LEAD_SPEED = 6         
param TRIGGER_DIST = 15      
param SAFETY_DIST = 12       

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_INIT_SPEED) until \
        (self.lane != ego.lane)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_END_SPEED)

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior EgoBehavior(target_section):
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to adversary) < globalParameters.TRIGGER_DIST

    do LaneChangeBehavior(laneSectionToSwitch=target_section, target_speed=globalParameters.EGO_SPEED)

    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to lead) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(1.0)

## 3. GEOMETRY
# A multi-lane straight road.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 50

checkPt = new OrientedPoint left of spawnPt by 3.5
targetSection = network.laneSectionAt(checkPt)

require targetSection is not None
require targetSection.lane != initLane

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is initially in front of the ego vehicle, then becomes adjacent to the ego's side during the bypass attempt.

param ADV_DIST = Range(15, 20)
param LEAD_DIST = Range(35, 45)

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

adv_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.ADV_DIST
adversary = new Car at adv_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior AdversaryBehavior()

lead_spot = new OrientedPoint following roadDirection from checkPt for globalParameters.LEAD_DIST
lead = new Car at lead_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

require (distance from adversary to intersection) > 20
require (distance from lead to intersection) > 20