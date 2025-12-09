# 1. MAP AND MODEL CONFIGURATION
param map = localPath('../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Constants
param EGO_SPEED = 12
param LEAD_SPEED = 6
param ADV_SPEED = 16
param LC_TRIGGER_DIST = 15
param ADV_TRIGGER_DIST = 20

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior AdversaryBehavior(target_section):
    # Drive fast in the adjacent lane
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance to ego) < globalParameters.ADV_TRIGGER_DIST
    
    # Block the ego's lane change maneuver by merging into its original lane
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.ADV_SPEED)
        
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    # Follow the slow lead vehicle
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to lead) < globalParameters.LC_TRIGGER_DIST

    # Attempt to overtake by changing lanes
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

# 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# 1. Find all lanes that have a lane to their left
lanes_with_left = filter(lambda l: l.sections[0]._laneToLeft is not None, network.lanes)

# 2. Pick a random lane from the filtered list and a point on it
initLane = Uniform(*lanes_with_left)
spawnPt = new OrientedPoint on initLane.centerline

# 3. Ensure the spawn point is not too close to an intersection
require (distance from spawnPt to intersection) > 80

# 4. Get the specific lane sections for the behaviors
ego_start_section = network.laneSectionAt(spawnPt)

# We know a left lane exists, but we must verify the section at this point is valid
require ego_start_section._laneToLeft is not None
ego_target_section = ego_start_section.laneToLeft
adv_lane = ego_target_section.lane

# 5. Define the adversary's target section (Ego's original lane)
adv_target_section = ego_start_section

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param LEAD_DIST = Range(20, 25)
param ADV_OFFSET = Range(-15, -5) # Adversary starts behind or alongside ego

# Spawn the slow lead vehicle ahead of the ego
lead_pt = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DIST
lead = new Car at lead_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

# Spawn the ego vehicle
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_target_section)

# Spawn the fast-moving adversary in the adjacent lane
adv_init_pt = new OrientedPoint on adv_lane.centerline
adv_base_pt = new OrientedPoint following roadDirection from adv_init_pt for globalParameters.ADV_OFFSET

# Align the adversary longitudinally with the ego's coordinate system
adv_spawn_pt = new OrientedPoint at adv_base_pt offset by (0, 0, 0.5)

adversary = new Car at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_target_section)

# Final safety checks
require (distance from lead to intersection) > 20
require (distance from adversary to intersection) > 20
terminate when (distance to ego) > 100