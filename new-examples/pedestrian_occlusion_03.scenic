# Scenario: A pedestrian hidden behind a parked truck sprints across the ego's lane at a sampled
# high speed once triggered, dramatically reducing the ego's available reaction and braking time.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.tesla.model3"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Pedestrian sprints at a sampled high speed and is triggered at a sampled close distance for minimum warning
param PED_SPEED = Range(4.0, 6.0)
param TRIGGER_DIST = Range(8, 13)

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
# Ego drives at a higher speed to further compress the available reaction window after pedestrian trigger
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=14)
require (distance to intersection) > 40

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Truck is placed at a sampled forward distance; running pedestrian hides behind it at a fixed lateral offset
param OCCLUDER_DIST = Range(18, 28)
param SIDE_OFFSET = -3.5

truck_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.OCCLUDER_DIST

occluder_truck = new Truck at truck_spot offset by (globalParameters.SIDE_OFFSET, 0, 0.5),
    with behavior ParkedBehavior()

ped_spawn = new OrientedPoint at (-0.5, 4.0) relative to occluder_truck,
    facing 90 deg relative to occluder_truck

bad_pedestrian = new Pedestrian at ped_spawn,
    with regionContainedIn None,
    with behavior PedestrianOcclusionBehavior(ego)

require (distance from bad_pedestrian to occluder_truck) > 1