# Scenario: A stationary bus on the shoulder provides a much larger occlusion volume than a truck,
# hiding a slow-walking elderly pedestrian who drifts into the lane at a fixed trigger distance,
# testing the ego's ability to detect and react to a low-speed crossing threat behind a large obstacle.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.tesla.model3"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Pedestrian moves at a fixed slow walking speed and is triggered at a fixed moderate distance from ego
param PED_SPEED = 1.2
param TRIGGER_DIST = 16

behavior CrossingWalk(target_speed):
    while True:
        take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(target_speed)

behavior PedestrianOcclusionBehavior(target_ego):
    while (distance from target_ego to self) > globalParameters.TRIGGER_DIST:
        wait
    do CrossingWalk(target_speed=globalParameters.PED_SPEED)

behavior ParkedBehavior():
    take SetBrakeAction(1.0)
    take SetHandBrakeAction(True)

# 3. ROAD GEOMETRY
# Ego drives at a moderate speed; the large bus occluder completely hides the slow pedestrian until trigger
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=10)
require (distance to intersection) > 40

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Bus is placed at a sampled forward distance on the left shoulder; pedestrian hides at its far side
param OCCLUDER_DIST = Range(22, 32)
param SIDE_OFFSET = -3.5

bus_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OCCLUDER_DIST

occluder_bus = new Car at bus_spot offset by (globalParameters.SIDE_OFFSET, 0, 0.5),
    with blueprint "vehicle.mitsubishi.fusorosa",
    with behavior ParkedBehavior()

ped_spawn = new OrientedPoint at (-0.5, 5.0) relative to occluder_bus,
    facing 90 deg relative to occluder_bus

bad_pedestrian = new Pedestrian at ped_spawn,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego)

require (distance from bad_pedestrian to occluder_bus) > 1