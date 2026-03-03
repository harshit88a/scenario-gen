# Scenario: The ego vehicle attempts to change to the right lane; the adversarial car is driving parallel to the ego in the target lane, blocking its path.


## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car maintains a parallel position alongside the ego in the right target lane, matching the ego's speed to continuously block the lane change attempt and force the ego to brake.

param EGO_SPEED = 12
param ADV_SPEED = 12
param SAFETY_DIST = 8

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for 3 seconds
    rightSection = network.laneSectionAt(self)._laneToRight
    try:
        do LaneChangeBehavior(laneSectionToSwitch=rightSection, target_speed=globalParameters.EGO_SPEED)
    interrupt when (distance to adversary) < globalParameters.SAFETY_DIST:
        take SetBrakeAction(0.8)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
# A multi-lane straight road where the ego occupies a lane and the adversarial vehicle occupies the right adjacent lane in parallel.

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
advPt = new OrientedPoint on rightLane.centerline

require (distance from spawnPt to intersection) > 60
require (distance from advPt to intersection) > 40

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car spawns in the right target lane at the same longitudinal position as the ego, creating a direct side-by-side blockage at the moment the ego initiates the lane change.

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

adversary = new Car at advPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior AdversaryBehavior()

require (distance from ego to adversary) < 8