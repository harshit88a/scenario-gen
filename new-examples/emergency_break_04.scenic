# Scenario: Lead Vehicle Emergency Brake due to Obstacle - The leading vehicle decelerates suddenly due to a trash bin, forcing the ego-vehicle to perform an emergency brake.


## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The leading vehicle drives until it detects an obstacle, then applies full brakes.

param LEAD_SPEED = Range(10, 12)
param OBSTACLE_BRAKE_DIST = Range(10, 15)
param BRAKE_FORCE = 1.0

behavior LeadCarBehavior(target_obstacle, brake_intensity):
    do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED) until \
        (distance from target_obstacle to self) < globalParameters.OBSTACLE_BRAKE_DIST
    
    while True:
        take SetBrakeAction(brake_intensity)


## 3. GEOMETRY
### The ego vehicle is initialized on a straight lane away from intersections.


EGO_SPEED = 12

lane = Uniform(*network.lanes)

EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 0.5),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 80


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### The adversarial lead car and a static obstacle are placed in sequence ahead of the ego vehicle.

param DIST_EGO_TO_LEAD = Range(10, 15)
param DIST_LEAD_TO_OBSTACLE = Range(30, 40)

lead_spot = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DIST_EGO_TO_LEAD

obs_spot = new OrientedPoint following roadDirection from lead_spot for globalParameters.DIST_LEAD_TO_OBSTACLE

obstacle = new Prop at obs_spot offset by (0, 0, 0.5),
    with blueprint "static.prop.trashcan01"

leadCar = new Car at lead_spot offset by (0, 0, 0.5),
    with behavior LeadCarBehavior(obstacle, globalParameters.BRAKE_FORCE)

terminate when ego.speed < 0.1 and (distance to leadCar) < 5