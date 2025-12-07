# Scenario: Ego vehicle attempts to overtake a slow lead vehicle by changing lanes, but an adversary vehicle in the target lane blocks the maneuver, forcing the ego to slow down.

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
param ADV_TRIGGER_DIST = 15

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

behavior AdversaryBehavior(target_section):
    # Drive in the target lane until close to Ego
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until \
        (distance to ego) < globalParameters.ADV_TRIGGER_DIST
    
    # Merge into Ego's original lane (Blocking the return/lane)
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.ADV_SPEED)
        
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    # Follow lead until close
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to lead) < globalParameters.LC_TRIGGER_DIST

    # Attempt lane change to avoid lead
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)

    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

# 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# 1. Filter lanes to find ONLY those with a valid Left Neighbor
# We check '_laneToLeft' (internal) to avoid crashing if it doesn't exist
lanes_with_left = filter(lambda l: l.sections[0]._laneToLeft is not None, network.lanes)

# 2. Pick a valid lane and point
initLane = Uniform(*lanes_with_left)
spawnPt = new OrientedPoint on initLane.centerline

# 3. Filter point to ensure safe distance from intersections
require (distance from spawnPt to intersection) > 80

# 4. Get the topological sections safely
ego_start_section = network.laneSectionAt(spawnPt)

# Since we pre-filtered, we can access the neighbor safely, 
# but we still require the specific section at spawnPt to be valid.
require ego_start_section._laneToLeft is not None

ego_target_section = ego_start_section.laneToLeft
adv_lane = ego_target_section.lane

# 5. Define Adversary's geometry (Targeting Ego's start lane)
adv_target_section = ego_start_section

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param LEAD_DIST = Range(20, 25)
param ADV_OFFSET = Range(-5, -10)

# Spawn Lead
lead_pt = new OrientedPoint following roadDirection from spawnPt for globalParameters.LEAD_DIST
lead = new Car at lead_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior LeadBehavior()

# Spawn Ego
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(ego_target_section)

# Spawn Adversary
# Use the centerline of the adv_lane to ensure valid placement
adv_init_pt = new OrientedPoint on adv_lane.centerline
adv_base_pt = new OrientedPoint following roadDirection from adv_init_pt for globalParameters.ADV_OFFSET

# Match longitudinal alignment with Ego
adv_spawn_pt = new OrientedPoint at adv_base_pt offset by (0, 0, 0.5)

adversary = new Car at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_target_section)

# Final checks
require (distance from lead to intersection) > 20
require (distance from adversary to intersection) > 20
terminate when (distance to ego) > 100