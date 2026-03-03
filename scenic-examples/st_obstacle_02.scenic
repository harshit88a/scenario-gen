# Scenario: The ego vehicle is driving on a straight road; the adversarial pedestrian stands behind a bus stop on the right front, then suddenly sprints out onto the road in front of the ego vehicle and stops.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. PARAMETERS

param EGO_SPEED = 10          # m/s  (~36 km/h)
param PED_SPRINT_SPEED = 4.5  # m/s  – fast sprint from behind bus stop
param TRIGGER_DIST = 20       # m    – distance at which pedestrian begins to sprint
param STOP_DIST = 3           # m    – distance at which pedestrian freezes in ego's path

## 3. BEHAVIORS

# The pedestrian waits (hidden behind the bus stop) until the ego is close enough,
# then sprints across the road and stops directly in the ego's path.
behavior PedestrianBusStopAmbushBehavior(sprint_speed, trigger_dist, stop_dist):
    # Phase 1 – waiting phase: stand still behind the bus stop
    while (distance from self to ego) > trigger_dist:
        take SetWalkingSpeedAction(0)

    # Phase 2 – sprint phase: dash onto the road toward ego's lane
    while True:
        if (distance from self to ego) <= stop_dist:
            # Phase 3 – stop dead in the road
            take SetWalkingSpeedAction(0)
            break
        else:
            take SetWalkingDirectionAction(self.heading)
            take SetWalkingSpeedAction(sprint_speed)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 4. GEOMETRY
# Select a straight lane well away from any intersection.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 70

## 5. SPAWN POSITIONS

# The pedestrian is placed further to the right than a normal roadside pedestrian to
# simulate being concealed behind a bus-stop shelter before sprinting out.
param START_DISTANCE = Range(22, 30)   # how far ahead of ego the bus stop sits
param LATERAL_OFFSET = Range(4.5, 6.0) # extra rightward offset to mimic bus-stop occlusion

# ── Ego vehicle ──────────────────────────────────────────────────────────────
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# ── Reference point at the bus-stop location ─────────────────────────────────
# Positioned ahead and to the right along the road centre-line.
bus_stop_ref = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

# ── Pedestrian spawn – tucked behind the bus stop (right side, extra lateral offset) ─
ped_spawn = new OrientedPoint right of bus_stop_ref by globalParameters.LATERAL_OFFSET

# The pedestrian faces 90 ° left (into the road) so that, when the sprint begins,
# the walking direction carries them directly across the ego's lane.
ped = new Pedestrian at ped_spawn offset by (0, 0, 1),
    facing 90 deg relative to bus_stop_ref.heading,
    with regionContainedIn None,
    with behavior PedestrianBusStopAmbushBehavior(
        globalParameters.PED_SPRINT_SPEED,
        globalParameters.TRIGGER_DIST,
        globalParameters.STOP_DIST
    )

## 6. VALIDITY CONSTRAINTS

# Pedestrian must start outside the drivable region (i.e., truly off-road / at bus stop).
require ped not in network.drivableRegion
# Ensure the chosen lane is long enough for the scenario to play out.
require (distance from spawnPt to initLane.centerline[-1]) > 50