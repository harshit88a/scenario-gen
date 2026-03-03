# Scenario: Pedestrian Crossing from Behind Occlusion - A parked truck occludes the view of a pedestrian who jaywalks into the ego vehicle's path when the ego gets close.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.tesla.model3"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial pedestrian jaywalks into the road, moving directly into the ego vehicle's path as it approaches.
param PED_SPEED = Range(1.5, 3.0)
param TRIGGER_DIST = Range(10, 18)

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


## 3. GEOMETRY
### A straight road featuring a parking lane or shoulder occupied by a large vehicle (truck).
lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline
ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=10)
require (distance to intersection) > 40


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The adversarial pedestrian is positioned behind a parked truck on the side of the road, initially occluded from the ego vehicle's line of sight.
param OCCLUDER_DIST = Range(20, 30) 
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