# Scenario: The ego vehicle is driving on a straight road; the adversarial pedestrian
# suddenly appears from behind a parked car on the right front and suddenly stops
# in front of the ego vehicle.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. PARAMETERS

param EGO_SPEED = 10             # m/s  (~36 km/h)
param PED_EMERGE_SPEED = 3.5     # m/s  – quick dash from behind parked car
param TRIGGER_DIST = 20          # m    – ego distance that triggers pedestrian emergence
param STOP_DIST = 3.5            # m    – ego distance at which pedestrian freezes

## 3. BEHAVIORS

# The parked car occludes the pedestrian until the ego is close.
# Once triggered the pedestrian dashes out from behind the parked car
# into the road and then freezes abruptly in the ego's path.
behavior PedestrianParkedCarAmbushBehavior(emerge_speed, trigger_dist, stop_dist):

    # Phase 1 – wait hidden behind the parked car
    while (distance from self to ego) > trigger_dist:
        take SetWalkingSpeedAction(0)

    # Phase 2 – dash out onto the road (leftward, into ego's lane)
    while True:
        if (distance from self to ego) <= stop_dist:
            # Phase 3 – sudden freeze in the road
            take SetWalkingSpeedAction(0)
            break
        else:
            take SetWalkingDirectionAction(self.heading)
            take SetWalkingSpeedAction(emerge_speed)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 4. BEHAVIORS FOR PARKED CAR
# The parked car is static – no behavior needed; it acts purely as an occlusion prop.

## 5. GEOMETRY
# Select a straight lane section well away from any intersection.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 70

## 6. SPAWN POSITIONS

param START_DISTANCE    = Range(20, 28)  # how far ahead the parked car sits
param CAR_LATERAL_OFFSET = Range(3.0, 4.0) # parked car right of lane centre (road shoulder)
param PED_HIDE_OFFSET   = Range(1.5, 2.5)  # pedestrian tucked behind rear of parked car
param PED_LATERAL_OFFSET = Range(4.0, 5.0) # pedestrian rightward offset behind the car

# ── Ego vehicle ───────────────────────────────────────────────────────────────
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# ── Reference point at the parked car location ───────────────────────────────
parked_car_ref = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

# ── Parked car – static vehicle on the right shoulder acting as occlusion ─────
# Slightly offset rearward from the reference so the pedestrian can hide behind it.
parked_car = new Car right of parked_car_ref by globalParameters.CAR_LATERAL_OFFSET,
    with blueprint Uniform(
        "vehicle.tesla.model3",
        "vehicle.toyota.prius",
        "vehicle.audi.a2"
    ),
    with behavior None

# ── Pedestrian spawn – hidden behind the rear of the parked car ───────────────
# The rearward longitudinal offset (PED_HIDE_OFFSET) places the pedestrian behind
# the parked car's rear bumper, fully occluded from the approaching ego vehicle.
# The extra lateral offset (PED_LATERAL_OFFSET) keeps them off the drivable surface
# at spawn so the validity constraint is satisfied.
ped_spawn = new OrientedPoint right of parked_car_ref by globalParameters.PED_LATERAL_OFFSET

ped = new Pedestrian at ped_spawn offset by (-globalParameters.PED_HIDE_OFFSET, 0, 1),
    facing 90 deg relative to parked_car_ref.heading,  # faces leftward into the road
    with regionContainedIn None,
    with behavior PedestrianParkedCarAmbushBehavior(
        globalParameters.PED_EMERGE_SPEED,
        globalParameters.TRIGGER_DIST,
        globalParameters.STOP_DIST
    )

## 7. VALIDITY CONSTRAINTS

# Pedestrian must start off the drivable region (behind / beside the parked car).
require ped not in network.drivableRegion
# Parked car must be off the main drivable lane (on the shoulder / parking strip).
require parked_car not in network.drivableRegion
# Lane must be long enough for the full scenario to play out.
require (distance from spawnPt to initLane.centerline[-1]) > 50