# Scenario: The ego vehicle is driving on a straight road; the adversarial pedestrian
# appears from a driveway on the left and suddenly stops, then walks diagonally
# across the road in front of the ego vehicle.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. PARAMETERS

param EGO_SPEED = 10             # m/s  (~36 km/h)
param PED_INITIAL_SPEED = 3.0    # m/s  – brisk emergence speed from driveway
param PED_DIAGONAL_SPEED = 1.5   # m/s  – slower diagonal walk across the road
param TRIGGER_DIST = 22          # m    – distance at which pedestrian begins to move
param STOP_DURATION = 30         # ticks – how long the pedestrian freezes mid-road
param DIAGONAL_ANGLE = 45        # deg  – angle of diagonal walk relative to road axis

## 3. BEHAVIORS

# The pedestrian emerges from the left driveway, halts abruptly in the road,
# then resumes walking at a diagonal angle across the ego's path.
behavior PedestrianDrivewayDiagonalBehavior(
        initial_speed, diagonal_speed,
        trigger_dist, stop_duration, diagonal_angle):

    # Phase 1 – idle in the driveway mouth until ego is close enough
    while (distance from self to ego) > trigger_dist:
        take SetWalkingSpeedAction(0)

    # Phase 2 – emerge onto the road heading rightward (across traffic)
    while (distance from self to ego) > (trigger_dist - 6):
        take SetWalkingDirectionAction(self.heading)
        take SetWalkingSpeedAction(initial_speed)

    # Phase 3 – sudden stop in the middle of the road
    for _ in range(stop_duration):
        take SetWalkingSpeedAction(0)

    # Phase 4 – resume walking diagonally (forward-right relative to road direction)
    # A positive diagonal_angle tilts the walk direction toward the ego's lane ahead.
    diag_heading = self.heading + diagonal_angle deg
    while True:
        take SetWalkingDirectionAction(diag_heading)
        take SetWalkingSpeedAction(diagonal_speed)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 4. GEOMETRY
# Select a straight lane section well away from any intersection.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 70

## 5. SPAWN POSITIONS

# The driveway mouth is simulated by spawning the pedestrian to the LEFT of the road,
# a moderate distance ahead of the ego, set back slightly to mimic being just inside
# a driveway opening before stepping onto the carriageway.
param START_DISTANCE  = Range(18, 26)  # how far ahead of ego the driveway sits
param LATERAL_OFFSET  = Range(4.0, 5.5) # leftward offset – driveway / verge position
param DRIVEWAY_SETBACK = Range(1.0, 2.0) # rearward nudge to simulate driveway depth

# ── Ego vehicle ───────────────────────────────────────────────────────────────
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# ── Reference point at the driveway mouth on the left ────────────────────────
driveway_ref = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

# ── Pedestrian spawn – just inside the driveway on the left ──────────────────
# 'left of' places the pedestrian on the left-hand verge; the rearward setback
# simulates depth into the driveway so the pedestrian is initially out of sight.
ped_spawn = new OrientedPoint left of driveway_ref by globalParameters.LATERAL_OFFSET,
    facing driveway_ref.heading

ped = new Pedestrian at ped_spawn offset by (-globalParameters.DRIVEWAY_SETBACK, 0, 1),
    facing 270 deg relative to driveway_ref.heading,  # facing rightward into the road
    with regionContainedIn None,
    with behavior PedestrianDrivewayDiagonalBehavior(
        globalParameters.PED_INITIAL_SPEED,
        globalParameters.PED_DIAGONAL_SPEED,
        globalParameters.TRIGGER_DIST,
        globalParameters.STOP_DURATION,
        globalParameters.DIAGONAL_ANGLE
    )

## 6. VALIDITY CONSTRAINTS

# Pedestrian must start off the drivable region (inside driveway / verge).
require ped not in network.drivableRegion
# Lane must be long enough for the full scenario sequence to unfold.
require (distance from spawnPt to initLane.centerline[-1]) > 50