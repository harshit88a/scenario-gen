# Scenario: Two pedestrians hide behind a parked truck and emerge in sequence at different sampled trigger
# distances, forcing the ego to handle a compound occlusion threat where a second pedestrian appears
# just as the ego begins reacting to the first.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.tesla.model3"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# First pedestrian triggers at a farther sampled distance; second triggers at a closer sampled distance
param PED_SPEED = Range(1.5, 3.0)
param TRIGGER_DIST_1 = Range(16, 22)
param TRIGGER_DIST_2 = Range(8, 13)

behavior CrossingWalk(target_speed):
    while True:
        take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(target_speed)

behavior PedestrianOcclusionBehavior(target_ego, trigger_dist):
    while (distance from target_ego to self) > trigger_dist:
        wait
    do CrossingWalk(target_speed=globalParameters.PED_SPEED)

behavior ParkedBehavior():
    take SetBrakeAction(1.0)
    take SetHandBrakeAction(True)

# 3. ROAD GEOMETRY
# Ego drives straight at a steady speed directly into the staggered dual-pedestrian hazard zone
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=10)
require (distance to intersection) > 40

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Both pedestrians hide behind the same truck; first crosses ahead of second at a staggered lateral offset
param OCCLUDER_DIST = Range(20, 30)
param SIDE_OFFSET = -3.5

truck_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OCCLUDER_DIST

occluder_truck = new Truck at truck_spot offset by (globalParameters.SIDE_OFFSET, 0, 0.5),
    with behavior ParkedBehavior()

ped_spawn_1 = new OrientedPoint at (-0.5, 4.0) relative to occluder_truck,
    facing 90 deg relative to occluder_truck

ped_spawn_2 = new OrientedPoint at (-0.5, 6.5) relative to occluder_truck,
    facing 90 deg relative to occluder_truck

pedestrian_1 = new Pedestrian at ped_spawn_1,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego, globalParameters.TRIGGER_DIST_1)

pedestrian_2 = new Pedestrian at ped_spawn_2,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego, globalParameters.TRIGGER_DIST_2)

require (distance from pedestrian_1 to occluder_truck) > 1
require (distance from pedestrian_2 to occluder_truck) > 1