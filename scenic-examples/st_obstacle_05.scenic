# Scenario: The ego vehicle is driving on a straight road; the adversarial pedestrian
# is hidden behind a vending machine on the right front, then abruptly dashes out
# onto the road and stops directly in the path of the ego vehicle.

## 1. MAP AND MODEL CONFIGURATION

param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. PARAMETERS

param EGO_SPEED = 10             # m/s  (~36 km/h)
param PED_DASH_SPEED = 4.0       # m/s  – abrupt sprint from behind vending machine
param TRIGGER_DIST = 20          # m    – ego distance that triggers pedestrian dash
param STOP_DIST = 3.0            # m    – ego distance at which pedestrian freezes

## 3. BEHAVIORS

# The pedestrian waits motionless behind the vending machine while the ego approaches.
# The moment the ego crosses the trigger threshold, the pedestrian sprints out onto
# the road and freezes dead in the ego's path.
behavior PedestrianVendingMachineAmbushBehavior(dash_speed, trigger_dist, stop_dist):

    # Phase 1 – stand still, concealed behind the vending machine
    while (distance from self to ego) > trigger_dist:
        take SetWalkingSpeedAction(0)

    # Phase 2 – abrupt dash onto the road (leftward into ego's lane)
    while True:
        if (distance from self to ego) <= stop_dist:
            # Phase 3 – freeze directly in the ego's path
            take SetWalkingSpeedAction(0)
            break
        else:
            take SetWalkingDirectionAction(self.heading)
            take SetWalkingSpeedAction(dash_speed)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

## 4. GEOMETRY
# Select a straight lane section well clear of any intersection.

initLane = Uniform(*network.lanes)
spawnPt = new OrientedPoint on initLane.centerline

require (distance from spawnPt to intersection) > 70

## 5. SPAWN POSITIONS

# The vending machine sits on the right-hand pavement/shoulder ahead of the ego.
# The pedestrian is tucked tightly behind it (rearward + lateral offsets) so they
# are fully occluded from the ego's perspective at scenario start.
param START_DISTANCE      = Range(20, 28)  # how far ahead the vending machine sits
param VM_LATERAL_OFFSET   = Range(3.5, 4.5) # vending machine rightward offset from lane centre
param PED_HIDE_OFFSET     = Range(0.6, 1.0) # rearward nudge behind the machine body
param PED_LATERAL_OFFSET  = Range(4.2, 5.2) # pedestrian rightward offset (behind machine flank)

# ── Ego vehicle ───────────────────────────────────────────────────────────────
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

# ── Reference point at the vending machine location ──────────────────────────
vm_ref = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

# ── Vending machine – static prop on the right pavement acting as occlusion ──
# CARLA blueprint: static.prop.vendingmachine
# Oriented parallel to the road so its broad face shields the pedestrian behind it.
vending_machine = new Prop right of vm_ref by globalParameters.VM_LATERAL_OFFSET,
    with blueprint "static.prop.vendingmachine",
    facing vm_ref.heading,         # broad face parallel to road – maximises occlusion
    with regionContainedIn None

# ── Pedestrian spawn – hidden tightly behind the vending machine ──────────────
# PED_LATERAL_OFFSET places the pedestrian beside the machine's far flank.
# The rearward PED_HIDE_OFFSET tucks them behind the rear edge so the ego cannot
# see them until they step out.
ped_spawn = new OrientedPoint right of vm_ref by globalParameters.PED_LATERAL_OFFSET

ped = new Pedestrian at ped_spawn offset by (-globalParameters.PED_HIDE_OFFSET, 0, 1),
    facing 90 deg relative to vm_ref.heading,   # faces leftward directly into the road
    with regionContainedIn None,
    with behavior PedestrianVendingMachineAmbushBehavior(
        globalParameters.PED_DASH_SPEED,
        globalParameters.TRIGGER_DIST,
        globalParameters.STOP_DIST
    )

## 6. VALIDITY CONSTRAINTS

# Pedestrian must start off the drivable region (concealed on the pavement).
require ped not in network.drivableRegion
# Vending machine must sit on the pavement, not on the travel lane.
require vending_machine not in network.drivableRegion
# Lane must be long enough for the full trigger–dash–stop sequence to play out.
require (distance from spawnPt to initLane.centerline[-1]) > 50