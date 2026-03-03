# Scenario: Control Loss Due to Road Conditions - The ego-vehicle encounters unexpected debris (crushed boxes) on the road, simulating bad conditions that force a reaction.

## 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr') 
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"


## 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
### The adversarial crushed boxes (static debris) act as a road hazard, forcing the ego vehicle to perform an emergency maneuver or brake.
param DEBRIS_START_DIST = Range(15, 20)
param DEBRIS_GAP = Range(6, 10) 

behavior StaticObjectBehavior():
    while True:
        wait


## 3. GEOMETRY
### The ego vehicle is placed on a straight section of the lane with a specific Z-offset to ensure stable initialization
EGO_SPEED = 10

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 50


## 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
### Multiple debris props are spawned in a sequence along the ego vehicle's path to create a hazardous road condition
debris_spot_1 = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris1 = new Prop at debris_spot_1,
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_2 = new OrientedPoint following roadDirection from debris_spot_1 for globalParameters.DEBRIS_GAP

debris2 = new Prop at debris_spot_2 offset by (Range(-0.5, 0.5), 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

debris_spot_3 = new OrientedPoint following roadDirection from debris_spot_2 for globalParameters.DEBRIS_GAP

debris3 = new Prop at debris_spot_3 offset by (Range(-0.5, 0.5), 0, 0),
    with blueprint "static.prop.creasedbox01",
    with physics True,
    with behavior StaticObjectBehavior()

terminate when (distance from ego to EgoSpawnPt) > 70