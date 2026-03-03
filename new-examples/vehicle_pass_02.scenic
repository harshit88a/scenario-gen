# Scenario: Ego bypasses a parked car using the opposite lane, yields to oncoming traffic, then encounters an oncoming motorcyclist that swerves into its lane, requiring emergency braking.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### Ego detects parked car, yields at the edge of its lane until the oncoming car clears, changes into the opposing lane to bypass, returns to original lane, then brakes hard when the motorcyclist swerves into its path. The oncoming car travels straight through. The motorcyclist follows its lane until it is close to the ego, then swerves into the ego's original lane.

param EGO_SPEED          = 10
param ONCOMING_SPEED     = Range(8, 12)
param MOTO_SPEED         = Range(10, 14)
param BYPASS_TRIGGER_DIST = 20
param BYPASS_CLEAR_DIST   = 15
param YIELD_DIST          = 30
param BRAKE_TRIGGER_DIST  = 18

behavior EgoBehavior(opposingSection, originalSection):
    # Phase 1: follow lane until close to parked car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) < globalParameters.BYPASS_TRIGGER_DIST

    # Phase 2: yield to oncoming car — hold position until it has cleared
    while (distance from self to oncomingCar) < globalParameters.YIELD_DIST:
        take SetBrakeAction(0.8)

    # Phase 3: change into opposing lane to bypass parked car
    do LaneChangeBehavior(laneSectionToSwitch=opposingSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 4: travel in opposing lane until fully past parked car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to parkedCar) > globalParameters.BYPASS_CLEAR_DIST

    # Phase 5: change back into original lane
    do LaneChangeBehavior(laneSectionToSwitch=originalSection,
                          target_speed=globalParameters.EGO_SPEED)

    # Phase 6: resume normal driving until motorcyclist swerves close
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance from self to motorcyclist) < globalParameters.BRAKE_TRIGGER_DIST

    # Phase 7: emergency brake to avoid collision with swerving motorcyclist
    while True:
        take SetBrakeAction(1.0)

behavior OncomingCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ONCOMING_SPEED)

behavior MotorcyclistBehavior(egoLaneSection):
    # Follow opposing lane until close enough to ego, then swerve into ego's lane
    do FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED) until \
        (distance from self to ego) < 25
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSection,
                          target_speed=globalParameters.MOTO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED)


## 3. GEOMETRY
### Ego is placed on a straight lane away from intersections. A left offset point resolves the opposing lane section for bypass. A right offset from that recovers the original lane section for return. Both are passed into EgoBehavior at construction time. The same original section is passed to the motorcyclist as its swerve target.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

leftCheckPt  = new OrientedPoint left of spawnPt by 3.5
opposingSection = network.laneSectionAt(leftCheckPt)

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
### Parked car blocks ego's lane. Oncoming car is placed in the opposing lane close enough to force a yield before the bypass. Motorcyclist is placed further in the opposing lane and swerves into ego's original lane after the bypass is complete.

param DIST_EGO_TO_PARKED     = Range(25, 30)
param DIST_EGO_TO_ONCOMING   = Range(35, 42)
param DIST_EGO_TO_MOTO       = Range(60, 70)

parked_spot  = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_PARKED
oncoming_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_ONCOMING
moto_spot    = new OrientedPoint following roadDirection from spawnPt for globalParameters.DIST_EGO_TO_MOTO

parkedCar = new Car at parked_spot offset by (0, 0, 0.5),
    with blueprint "vehicle.tesla.model3"

oncomingCar = new Car at oncoming_spot offset by (0, 0, 0.5),
    facing 180 deg relative to roadDirection,
    with behavior OncomingCarBehavior()

motorcyclist = new Car at moto_spot offset by (0, 0, 0.5),
    with blueprint "vehicle.harley-davidson.low_rider",
    facing 180 deg relative to roadDirection,
    with behavior MotorcyclistBehavior(originalSection)

require (distance from parkedCar to intersection) > 50

terminate when ego.speed < 0.5 and (distance from ego to motorcyclist) < 6