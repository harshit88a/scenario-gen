# Scenario: Ego bypasses a lane-blocking parked car using the opposite lane while monitoring oncoming traffic, then encounters a jaywalking pedestrian requiring emergency braking.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### Ego follows its lane, detects the parked car ahead, changes to the pre-computed left opposing lane section to bypass, then changes back to the pre-computed original lane section, and finally brakes for the jaywalking pedestrian. Oncoming car travels head-on. Pedestrian crosses from the right curb when ego is close.

param ONCOMING_SPEED      = Range(8, 12)
param EGO_SPEED           = 10
param BYPASS_TRIGGER_DIST = 20
param BYPASS_CLEAR_DIST   = 15

behavior EgoBehavior(opposingSection, originalSection):
    # Phase 1: follow lane until parked car is within trigger distance
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) < globalParameters.BYPASS_TRIGGER_DIST

    # Phase 2: change into opposing lane to bypass parked car
    do LaneChangeBehavior(laneSectionToSwitch=opposingSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 3: travel in opposing lane until clear of parked car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) > globalParameters.BYPASS_CLEAR_DIST

    # Phase 4: change back into original lane
    do LaneChangeBehavior(laneSectionToSwitch=originalSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 5: resume normal lane following until pedestrian encounter
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior OncomingCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)

behavior JaywalkBehavior():
    do CrossingBehavior(ego, min_speed=1.2, threshold=15)


## 3. GEOMETRY
### Ego is placed on a straight lane. A left offset point identifies the opposing lane section for bypass, and a right offset from that point recovers the original lane section for return. Both are passed to EgoBehavior at construction time.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

# Point 3.5m to the left identifies the opposing lane
leftCheckPt = new OrientedPoint left of spawnPt by 3.5
opposingSection = network.laneSectionAt(leftCheckPt)

# Point 3.5m back to the right from leftCheckPt recovers the original lane
rightCheckPt = new OrientedPoint right of leftCheckPt by 3.5
originalSection = network.laneSectionAt(rightCheckPt)

require opposingSection is not None
require getattr(opposingSection, 'lane', None) != initLane
require originalSection is not None

ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(opposingSection, originalSection)

require (distance to intersection) > 80


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The parked car blocks the ego's lane; the oncoming car approaches head-on in the opposing lane; the pedestrian starts on the right curb beyond the parked car and crosses into the ego's path after the bypass is complete.

param DIST_EGO_TO_PARKED     = Range(25, 30)
param DIST_EGO_TO_PEDESTRIAN = Range(50, 58)
param DIST_EGO_TO_ONCOMING   = Range(55, 65)

parked_spot     = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_PARKED
pedestrian_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_PEDESTRIAN
oncoming_spot   = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_ONCOMING

parkedCar = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint "vehicle.tesla.model3"

oncomingCar = new Car at oncoming_spot offset by (0, 0, 0.5),
    facing 180 deg relative to roadDirection,
    with behavior OncomingCarBehavior()

pedestrian = new Pedestrian at pedestrian_spot offset by (4, 0, 0.5),
    facing -90 deg relative to roadDirection,
    with regionContainedIn None,
    with behavior JaywalkBehavior()

require (distance from parkedCar to intersection) > 50

terminate when ego.speed < 0.5 and (distance from ego to pedestrian) < 5