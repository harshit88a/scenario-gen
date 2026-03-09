# Scenario: A parked van fully occludes a pedestrian who steps into the ego's path at a precisely
# fixed trigger distance, creating a fully deterministic occlusion jaywalking event with no randomness.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.tesla.model3"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Pedestrian waits motionless behind the van until ego reaches a fixed trigger distance, then walks into the lane
param PED_SPEED = 2.0
param TRIGGER_DIST = 14

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
# Ego cruises on a straight lane at a fixed speed well away from any intersection
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=10)
require (distance to intersection) > 40

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Parked van is placed at a fixed side offset ahead; pedestrian hides directly behind it at a fixed position
param OCCLUDER_DIST = 25
param SIDE_OFFSET = -3.5

van_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OCCLUDER_DIST

occluder_van = new Car at van_spot offset by (globalParameters.SIDE_OFFSET, 0, 0.5),
    with blueprint "vehicle.ford.ambulance",
    with behavior ParkedBehavior()

ped_spawn = new OrientedPoint at (-0.5, 4.0) relative to occluder_van,
    facing 90 deg relative to occluder_van

bad_pedestrian = new Pedestrian at ped_spawn,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego)

require (distance from bad_pedestrian to occluder_van) > 1