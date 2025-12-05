# Scenario: Ego Vehicle waits at 4-way intersection behind a stationary vehicle. An adversary vehicle in the adjacent right lane passes the Ego.
# The Ego then performs a lane change to the right to bypass the stationary vehicle.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model

# Constants
param EGO_SPEED = Range(7, 10)
param ADV_SPEED = 10
param BRAKE_INTENSITY = 1.0
param BYPASS_SAFETY_DIST = 10 

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT

# Behavior for the Stationary Vehicle (Obstacle)
behavior StationaryBehavior():
    take SetBrakeAction(1.0)

# Behavior for the Adversary (Just passing through)
behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    # 1. Wait while the adversary is passing
    while (distance to adversary) < globalParameters.BYPASS_SAFETY_DIST:
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)
    
    # 2. Perform Lane Change to the right
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)
    
    # 3. Continue driving
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. ROAD GEOMETRY
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

# A. Find a 4-way intersection
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

# B. Find a lane capable of a left turn that HAS a lane to its right
valid_lanes = filter(lambda l: l.sections[-1]._laneToRight is not None, intersec.incomingLanes)
stat_lane = Uniform(*valid_lanes)

# C. Define the Adjacent Right Lane (Adversary Lane)
stat_end_section = stat_lane.sections[-1]
adv_lane = stat_end_section.laneToRight.lane

# D. Define Trajectories
adv_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, adv_lane.maneuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
param STAT_DIST = Range(6, 9)       

# ADJUSTMENT: Increased gap to avoid bumper collision (Min 12m instead of 8m)
param EGO_GAP = Range(12, 15)        

param ADV_DIST = Range(20, 25)      

# 1. Spawn Stationary Car (The Obstacle)
stat_init_pt = new OrientedPoint in stat_lane.centerline
stat_spawn_pt = new OrientedPoint following roadDirection from stat_init_pt for -globalParameters.STAT_DIST

# 2. Spawn Ego (Behind Stationary)
ego_spawn_pt = new OrientedPoint following roadDirection from stat_spawn_pt for -globalParameters.EGO_GAP

# 3. Spawn Adversary (In right lane)
adv_init_pt = new OrientedPoint in adv_lane.centerline
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

# 4. PREPARE EGO BEHAVIOR ARGUMENTS
ego_start_section = network.laneSectionAt(ego_spawn_pt)

# Requirements
require ego_start_section is not None
require ego_start_section._laneToRight is not None

target_section = ego_start_section.laneToRight

# 5. Instantiate Agents
stationary = new Car at stat_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior StationaryBehavior()

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(target_section)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

# Requirements
require monitor GreenWave()
terminate when (distance to stat_spawn_pt) > 70