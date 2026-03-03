# Scenario: Ego bypasses a parked car using the opposite lane and encounters an oncoming car that suddenly turns into its path without signaling, requiring immediate evasive braking.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### Ego detects the parked car, changes into the opposing lane to bypass it, then encounters the oncoming car turning abruptly into its path and applies emergency brakes. The oncoming car drives normally until the ego is in the opposing lane and within a trigger distance, then executes an unsignaled turn into the ego's path by changing into the ego's original lane.

param EGO_SPEED           = 10
param ONCOMING_SPEED      = Range(8, 12)
param BYPASS_TRIGGER_DIST = 20
param BYPASS_CLEAR_DIST   = 15
param TURN_TRIGGER_DIST   = 22
param BRAKE_TRIGGER_DIST  = 15

behavior EgoBehavior(opposingSection, originalSection):
    # Phase 1: follow lane until close to parked car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) < globalParameters.BYPASS_TRIGGER_DIST

    # Phase 2: change into opposing lane to bypass parked car
    do LaneChangeBehavior(laneSectionToSwitch=opposingSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 3: travel in opposing lane until fully past parked car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) > globalParameters.BYPASS_CLEAR_DIST

    # Phase 4: attempt to return to original lane but oncoming car cuts in
    do LaneChangeBehavior(laneSectionToSwitch=originalSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 5: emergency brake when oncoming car is dangerously close
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to oncomingCar) < globalParameters.BRAKE_TRIGGER_DIST

    while True:
        take SetBrakeAction(1.0)

behavior OncomingCarBehavior(egoLaneSection):
    # Drive normally in opposing lane until ego is close enough, then cut unsignaled into ego's lane
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED) until \
        (distance from self to ego) < globalParameters.TURN_TRIGGER_DIST
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSection,
                          target_speed=globalParameters.ONCOMING_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)


## 3. GEOMETRY
### Ego is placed on a straight lane away from intersections. A left offset point resolves the opposing lane section for the bypass. A right offset from that recovers the original lane section for return. The original section is also passed to the oncoming car as its unsignaled turn target.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

leftCheckPt     = new OrientedPoint left of spawnPt by 3.5
opposingSection = network.laneSectionAt(leftCheckPt)

rightCheckPt    = new OrientedPoint right of leftCheckPt by 3.5
originalSection = network.laneSectionAt(rightCheckPt)

require opposingSection is not None
require getattr(opposingSection, 'lane', None) != initLane
require originalSection is not None

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(opposingSection, originalSection)

require (distance to intersection) > 80


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The parked car blocks the ego's lane ahead. The oncoming car is placed further ahead in the opposing lane travelling toward the ego and cuts into the ego's original lane without signaling when the ego is in the bypass corridor.

param DIST_EGO_TO_PARKED   = Range(25, 30)
param DIST_EGO_TO_ONCOMING = Range(50, 60)

parked_spot   = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_PARKED
oncoming_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_ONCOMING

parkedCar = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint "vehicle.tesla.model3"

oncomingCar = new Car at oncoming_spot offset by (0, 0, 0.5),
    facing 180 deg relative to roadDirection,
    with behavior OncomingCarBehavior(originalSection)

require (distance from parkedCar to intersection) > 50

terminate when ego.speed < 0.5 and (distance from ego to oncomingCar) < 6