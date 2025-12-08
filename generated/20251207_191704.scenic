# 1. MAP AND MODEL CONFIGURATION
param map = localPath('../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Constants
param EGO_SPEED = 12
param LEAD_SPEED = 7
param ADV_SPEED = 15
param LC_TRIGGER_DIST = 15

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior AdversaryBehavior():
    # Drive fast in the adjacent lane to block the ego's lane change attempt
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    # Follow the slow lead vehicle until close
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to lead) < globalParameters.LC_TRIGGER_DIST

    # Attempt to change lanes to overtake
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)

    # Continue in the new lane
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

# 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# 1. Filter lanes to find only those with a valid left neighbor
lanes_with_left = filter(lambda l: l.sections[0]._laneToLeft is not None, network.lanes)

# 2. Pick a valid lane and a point on it for the ego
initLane = Uniform(*lanes_with_left)
spawnPt = new OrientedPoint on initLane.centerline

# 3. Ensure the spawn point is far from any intersection
require (distance from spawnPt to intersection) > 80

# 4. Get the specific lane sections for the ego and the target lane
ego_start_section = network.laneSectionAt(spawnPt)

# We filtered for lanes with a left, but we must require the specific section has one
require ego_start_section._laneToLeft is not None
ego_target_section = ego_start_section.laneToLeft

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param LEAD_DIST = Range(20, 25)
param ADV_OFFSET = Range(-5, -10) # Adversary starts 5-10m behind ego

# Spawn Ego
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_target_section)

# Spawn Lead Vehicle ahead of Ego
lead_pt = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DIST
lead = new Car at lead_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

# Spawn Adversary in the adjacent lane, slightly behind Ego
# 1. Create a point in the adjacent lane, aligned with ego
adv_aligned_pt = new OrientedPoint left of spawnPt by 3.5
# 2. Move this point backwards along the road to its starting position
adv_spawn_pt = new OrientedPoint following roadDirection from adv_aligned_pt for globalParameters.ADV_OFFSET

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior()

# Final safety checks
require (distance from lead to intersection) > 20
require (distance from adversary to intersection) > 20
terminate when (distance to ego) > 100