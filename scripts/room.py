#/usr/bin/env python

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
import time
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'
robot = Robot ('hrp2_14')
robot.setTranslationBounds (-4, 4, -4, 4, 0, 1)
cl = robot.client
r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r(q0)

# Display furnitures in RViz
k=2.3
kk=k-1
r.addObject('lamp','lamp_base')
lamp_pos= (0-0.3,-k,0,1,0,0,0)
r.moveObject('lamp',lamp_pos)
r.addObject('armchair','armchair_base')
armchair_pos= (kk,kk,0,0.707,0,0,-0.707)
r.moveObject('armchair',armchair_pos)
r.addObject('pc','pc_base')
pc_pos= (k,-k+0.4,1.04,0.707,0,0,-0.707)
r.moveObject('pc',pc_pos)
r.addObject('desk','desk_base')
desk_pos= (k,-k,0,0.707,0,0,-0.707)
r.moveObject('desk',desk_pos)
r.addObject('chair','chair_base')
chair_pos= (kk,-k+0.4,0,0.707,0,0,-0.707)
r.moveObject('chair',chair_pos)
r.addObject('table','table_base')
table_pos= (0,kk,0,1,0,0,0)
r.moveObject('table',table_pos)
r.addObject('commode','commode_base')
commode_pos= (-k-0.2,-k+0.2,0,0.707,0,0,0.707)
r.moveObject('commode',commode_pos)
r.addObject('books','books_base')
books_pos= (-k+0.1,-k,1,0.707,0,0,0.707)
r.moveObject('books',books_pos)
r.addObject('deskLamp','deskLamp_base')
deskLamp_pos= (k+0.3,-k+0.2,0.98,0.707,0,0,-0.707)
r.moveObject('deskLamp',deskLamp_pos)
r.addObject('beer','beer_base')
beer_pos= (0,kk,0.28,1,0,0,0) # on the "table basse"
r.moveObject('beer',beer_pos)
r.addObject('stuff','stuff_base')
stuff_pos= (kk-0.2,-k-0.4,1.4,0.707,0,0,0.707)
r.moveObject('stuff',stuff_pos)
r(q0) # end of display

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)
cl.problem.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])
# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    cl.problem.lockDof (name, value)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    cl.problem.lockDof (name, value)

q1=[1, 2.2, 0.64, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q1proj = cl.problem.applyConstraints (q1)

q2=[0.5, -2.2, 0.64, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q2proj = cl.problem.applyConstraints (q2) # quat 0.5,0,0,-0.5 => assert here

# Build the problem
cl.problem.setInitialConfig (q1proj)
cl.problem.addGoalConfig (q2proj)
p = PathPlayer (cl, r)
cl.problem.solve () # assert raised on quaternions (getRotation for a collision test)

# Add obstacles in hpp
cl.obstacle.loadObstacleModel('room_description','room')
cl.obstacle.moveObstacle ('armchair_base', armchair_pos)
cl.obstacle.moveObstacle ('pc_base', pc_pos)
cl.obstacle.moveObstacle ('desk_base', desk_pos)
cl.obstacle.moveObstacle ('chair_base', chair_pos)
cl.obstacle.moveObstacle ('table_base', table_pos)
cl.obstacle.moveObstacle ('commode_base', commode_pos)
cl.obstacle.moveObstacle ('books_base', books_pos)
cl.obstacle.moveObstacle ('deskLamp_base', deskLamp_pos)
cl.obstacle.moveObstacle ('beer_base', beer_pos)
cl.obstacle.moveObstacle ('stuff_base', stuff_pos)

# Nodes from the roadmap
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)

## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(1)
r( cl.problem.configAtDistance(1,5) )
cl.problem.clearRoadmap ()
cl.problem.optimizePath (2)
cl.problem.directPath(q1,q2)
