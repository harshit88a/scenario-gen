# Scenario: Three construction cones are fanned out laterally across the full width of the lane
# at a fixed distance ahead, forming a wall-like blockade that eliminates any clear driving path
# and forces the ego to brake or swerve onto the shoulder.

# 1. MAP AND MODEL CONFIGURATION
param map = localPath('./../CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
param carla_map = 'Town05'
param use2DMap = True
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# 2. ADV BEHAVIOR OF THE SURROUNDING OBJECT
# Three cones are fully static and spread at fixed lateral positions to fully block the lane
param DEBRIS_START_DIST = 25
param LATERAL_SPREAD = 1.6

behavior StaticObjectBehavior():
    while True:
        wait

# 3. ROAD GEOMETRY
# Ego cruises straight toward the lateral blockade with no prior warning of the obstruction
EGO_SPEED = 13

lane = Uniform(*network.lanes)
EgoSpawnPt = new OrientedPoint on lane.centerline

ego = new Car at EgoSpawnPt offset by (0, 0, 1.0),
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

require (distance to intersection) > 50

# 4. RELATIVE SPAWN POSITION OF THE ADVERSARIAL AGENT
# Left, center, and right cones are placed at the same forward distance but spread across the lane width
debris_spot_center = new OrientedPoint following roadDirection from EgoSpawnPt for globalParameters.DEBRIS_START_DIST

debris_center = new Prop at debris_spot_center offset by (0, 0, 0),
    with blueprint "static.prop.constructioncone",
    with physics True,
    with behavior StaticObjectBehavior()

debris_left = new Prop at debris_spot_center offset by (globalParameters.LATERAL_SPREAD, 0, 0),
    with blueprint "static.prop.constructioncone",
    with physics True,
    with behavior StaticObjectBehavior()

debris_right = new Prop at debris_spot_center offset by (-globalParameters.LATERAL_SPREAD, 0, 0),
    with blueprint "static.prop.constructioncone",
    with physics True,
    with behavior StaticObjectBehavior()

terminate when (distance from ego to EgoSpawnPt) > 70