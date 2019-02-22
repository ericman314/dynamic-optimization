from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Point3, Vec3, ConfigVariableDouble, DirectionalLight, AmbientLight, VBase4
from panda3d.bullet import BulletWorld, BulletCylinderShape, BulletPlaneShape, BulletRigidBodyNode, Z_up

import imagery
import falcon

from math import pi, sin, cos
 
ConfigVariableDouble('default-far').setValue(20000000)

class MyApp(ShowBase):
 
  def __init__(self):

    ShowBase.__init__(self)

    # Set up the Bullet physics engine
    self.world = BulletWorld()
    self.world.setGravity(Vec3(0, 0, -9.81))

    # Make a ground for the rocket to land on
    groundNode = BulletRigidBodyNode('Ground')
    groundNode.addShape(BulletPlaneShape(Vec3(0, 0, 1), 1))
    
    self.render.attachNewNode(groundNode).setPos(0, 0, 0)

    # Make the rocket
    falconRadius = 1.8542
    falconHeight = 47
    rocketNode = BulletRigidBodyNode('Rocket')
    rocketNode.setMass(1.0)   # TODO: Update this, determine how Bullet reacts if the mass changes
    rocketNode.addShape(BulletCylinderShape(1.8542, 47, Z_up))   # radius, height, axis
    
    # NOTE: Possible future need to save return valve from attachNewNode
    rocketNP = self.render.attachNewNode(rocketNode)
    rocketNP.setPos(0, 0, 100)

    self.world.attachRigidBody(groundNode)
    self.world.attachRigidBody(rocketNode)


    # Load models
    imagery.loadScenery(self.render.attachNewNode('sceneryNode'))

    rocketModelNodePath = rocketNP.attachNewNode('falconNode')
    falcon.loadFalcon(rocketModelNodePath, falconRadius, falconHeight)

    # Lighting
    dlight = DirectionalLight('dlight')
    dlight.setColor(VBase4(0.8, 0.8, 0.8, 1))
    dlnp = self.render.attachNewNode(dlight)
    dlnp.setHpr(0, -60, 0)
    self.render.setLight(dlnp)

    alight = AmbientLight('alight')
    alight.setColor(VBase4(0.2, 0.2, 0.2, 1))
    alnp = self.render.attachNewNode(alight)
    self.render.setLight(alnp)

    # Add the spinCameraTask procedure to the task manager.
    self.taskMgr.add(self.tick, "Tick")


  # Perform the physics here. Also rotate the camera around.
  def tick(self, task):
    
    # Run Bullet simulation (where is globalClock defined? Well it works somehow...)
    dt = globalClock.getDt()
    self.world.doPhysics(dt)

    # Rotate camera
    angleDegrees = task.time * 5.0
    angleRadians = angleDegrees * (pi / 180.0)
    # dist = 1 * 1.5 ** (-10 * cos(task.time / 10) + 20)
    dist = 250
    self.camera.setPos(dist * sin(angleRadians), -dist * cos(angleRadians), dist*0.5)
    self.camera.setHpr(angleDegrees, -25, 0)
    
    return Task.cont    # Execute the task again




app = MyApp()
app.run()