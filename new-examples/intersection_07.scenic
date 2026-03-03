# Scenario: Ego Vehicle waits at 4-way intersection behind a stationary vehicle. An adversary vehicle in the adjacent right lane passes the Ego. The Ego then performs a lane change to the right to bypass the stationary vehicle.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# The adversarial car travels at a constant speed or accelerates to pass the ego vehicle on the right, creating a conflict when the ego attempts to change lanes.

param EGO_SPEED = Range(7, 10)
param ADV_SPEED = 10
param BRAKE_INTENSITY = 1.0
param BYPASS_SAFETY_DIST = 10 

behavior StationaryBehavior():
    take SetBrakeAction(1.0)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.ADV_SPEED)

behavior EgoBehavior(target_section):
    while (distance to adversary) < globalParameters.BYPASS_SAFETY_DIST:
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)
    
    do LaneChangeBehavior(
        laneSectionToSwitch=target_section,
        target_speed=globalParameters.EGO_SPEED)
    
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

monitor GreenWave():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

## 3. GEOMETRY
# A multi-lane approach to a four-way intersection.

ADV_MODEL = "vehicle.tesla.model3"

fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
intersec = Uniform(*fourWayIntersections)

valid_lanes = filter(lambda l: l.sections[-1]._laneToRight is not None, intersec.incomingLanes)
stat_lane = Uniform(*valid_lanes)

stat_end_section = stat_lane.sections[-1]
adv_lane = stat_end_section.laneToRight.lane

adv_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, adv_lane.maneuvers)
adv_maneuver = Uniform(*adv_maneuvers)
adv_trajectory = [adv_maneuver.startLane, adv_maneuver.connectingLane, adv_maneuver.endLane]

## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# The adversarial car is in the lane to the right of the ego vehicle, initially positioned at the rear or side.

param STAT_DIST = Range(6, 9)       
param EGO_GAP = Range(12, 15)        
param ADV_DIST = Range(20, 25)      

stat_init_pt = new OrientedPoint in stat_lane.centerline
stat_spawn_pt = new OrientedPoint following roadDirection from stat_init_pt for -globalParameters.STAT_DIST

ego_spawn_pt = new OrientedPoint following roadDirection from stat_spawn_pt for -globalParameters.EGO_GAP

adv_init_pt = new OrientedPoint in adv_lane.centerline
adv_spawn_pt = new OrientedPoint following roadDirection from adv_init_pt for -globalParameters.ADV_DIST

ego_start_section = network.laneSectionAt(ego_spawn_pt)

require ego_start_section is not None
require ego_start_section._laneToRight is not None

target_section = ego_start_section.laneToRight

stationary = new Car at stat_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior StationaryBehavior()

ego = new Car at ego_spawn_pt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(target_section)

adversary = new Car at adv_spawn_pt offset by (0, 0, 0.5),
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(adv_trajectory)

require monitor GreenWave()
terminate when (distance to stat_spawn_pt) > 70