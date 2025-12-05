# Scenario: Ego approaches a slow-moving leading vehicle and changes lanes to evade.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
param EGO_SPEED = 15
param ADV_SPEED = 6
param DIST_THRESHOLD = 15

# Adversary: Just drives slowly
behavior SlowLeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

# Ego: Follows lane, then changes lane when close to the adversary
# Note: target_section must be a LaneSection object, not a Lane object
behavior EgoBehavior(target_section):
    # 1. Drive normally until close to the slow car
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until \
        (distance to advAgent) < globalParameters.DIST_THRESHOLD
    
    # 2. Perform Lane Change
    # We pass the specific LaneSection we calculated in the Geometry step
    do LaneChangeBehavior(laneSectionToSwitch=target_section, target_speed=globalParameters.EGO_SPEED)
    
    # 3. Continue driving in new lane
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

## 3. GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Pick any lane from the network
initLane = Uniform(*network.lanes)

# Create a spawn point on this lane
spawnPt = new OrientedPoint on initLane.centerline

# -- SPATIAL QUERY FOR LEFT LANE SECTION --
# 1. Define a point 3.5m to the left (standard lane width)
checkPt = new OrientedPoint left of spawnPt by 3.5

# 2. Query the network for the *LaneSection* at that point
# This is the specific object required by LaneChangeBehavior
targetSection = network.laneSectionAt(checkPt)

# Requirement 1: A valid lane section must exist there
require targetSection is not None

# Requirement 2: The found section must actally be different (not just a wide lane)
require targetSection.lane != initLane

# Create Ego at the verified spawn point
# We pass the valid 'targetSection' to the behavior
ego = new Car at spawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetSection)

# Requirement 3: Avoid spawning too close to intersections
require (distance to intersection) > 50

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param START_DISTANCE = Range(20, 30)

# Spawn the adversary ahead of the ego
adv_spot = new OrientedPoint following roadDirection from spawnPt for globalParameters.START_DISTANCE

advAgent = new Car at adv_spot offset by (0, 0, 0.5),
    with behavior SlowLeadBehavior()

require (distance from advAgent to intersection) > 20