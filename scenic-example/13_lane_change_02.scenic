# Scenario: Ego vehicle performs a lane change to bypass a slow  adversary vehicle but cannot return to its original lane because 
# the adversary accelerates. Ego vehicle must then slow down to avoid  collision with leading vehicle in new lane.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Constants for speeds (m/s) and distances (m)
param EGO_SPEED = 12
param ADV_INIT_SPEED = 5
param ADV_END_SPEED = 15     # Adversary speeds up to block return
param LEAD_SPEED = 6         # Slow lead in the new lane
param TRIGGER_DIST = 15      # Distance to trigger lane change
param SAFETY_DIST = 12       # Distance to trigger braking behind lead

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

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 1. Pick a lane
initLane = Uniform(*network.lanes)

# 2. Pick a point on that lane
spawnPt = new OrientedPoint on initLane.centerline

# CRITICAL FIX: Filter this random point *immediately* before defining dependent variables
require (distance from spawnPt to intersection) > 50

# 3. Define the spatial query for the target lane (Left)
checkPt = new OrientedPoint left of spawnPt by 3.5
targetSection = network.laneSectionAt(checkPt)

# 4. Filter the target section
require targetSection is not None
require targetSection.lane != initLane

# 5. Spawn Ego
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param ADV_DIST = Range(15, 20)
param LEAD_DIST = Range(35, 45)

# Spawn Adversary
adv_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.ADV_DIST
adversary = new Car at adv_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior AdversaryBehavior()

# Spawn Lead
lead_spot = new OrientedPoint following roadDirection from checkPt for globalParameters.LEAD_DIST
lead = new Car at lead_spot offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

# Final checks for agents
require (distance from adversary to intersection) > 20
require (distance from lead to intersection) > 20