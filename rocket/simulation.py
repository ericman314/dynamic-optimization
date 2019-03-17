from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from direct.stdpy import threading
from panda3d.core import Point3, Vec3, ConfigVariableDouble, ConfigVariableInt, DirectionalLight, AmbientLight, VBase4, LVector3f, AntialiasAttrib, TransformState
from panda3d.bullet import BulletWorld, BulletCylinderShape, BulletPlaneShape, BulletRigidBodyNode, X_up, Y_up, Z_up, BulletGenericConstraint, BulletDebugNode
import time
import sys
import math

import imagery
import falcon

from math import pi, sin, cos
 
ConfigVariableDouble('default-far').setValue(20000000)
ConfigVariableInt('framebuffer-multisample').setValue(1)
ConfigVariableInt('multisamples').setValue(2)

class MyApp(ShowBase):
 
  def __init__(self):

    ShowBase.__init__(self)

    # Set up the Bullet physics engine
    self.world = BulletWorld()
    self.world.setGravity(Vec3(0, 0, -9.81))

    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(False)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = self.render.attachNewNode(debugNode)
    debugNP.show()
    
    self.world.setDebugNode(debugNP.node())

    # Make a ground for the rocket to land on
    groundNP = self.render.attachNewNode(BulletRigidBodyNode('Ground'))
    groundNP.setPos(0, 0, 0)
    groundNP.node().addShape(BulletPlaneShape(Vec3(0, 0, 1), 1))
    groundNP.node().set_restitution(0.5)

    self.world.attachRigidBody(groundNP.node())

    

    # Falcon 9 parameters
    f9ThrustSL = 7607000 / 9     # N, per engine
    f9ThrustVac = 8227000 / 9    # N, per engine
    f9MinimumThrottle = 0.57     # Minimum throttle capability
    f9PropMass = 417000          # mass of propellent when 100% full
    f9BodyMass = 27200           # mass of empty falcon 9 without propellent
    f9Radius = 1.8542     # Radius of F9 in meters
    f9Height = 47         # Height of F9 in meters
    f9COMoffset = -8            # Center of mass relative to center of body, in meters
    
    # Initial Conditions
    initPropLoad = 0.15 * f9PropMass    # Amount of full-tank propellent remaining
    initX = 0
    initY = 0
    initZ = 60
        
    # Make the rocket
    self.f9BodyNP = self.render.attachNewNode(BulletRigidBodyNode('RocketBody'))
    
    self.f9BodyNP.node().addShape(BulletCylinderShape(f9Radius, f9Height, Z_up), TransformState.makePos(Vec3(0, 0, -f9COMoffset)))   # radius, height, axis
    # Rocket landing legs
    
    for i in range(4):
      leg = BulletCylinderShape(0.1, 10, X_up)
      self.f9BodyNP.node().addShape(leg, TransformState.makePosHpr(Vec3(6*math.cos(i*math.pi/2),6*math.sin(i*math.pi/2),-f9Height*0.5-f9COMoffset),Vec3(i*90,0,30)))
    

    
    self.f9BodyNP.node().setMass(f9BodyMass)
    self.f9BodyNP.setPos(initX, initY, initZ)


    self.world.attachRigidBody(self.f9BodyNP.node())


    # We will create four cylinders: one for the rocket body, one for the engines, and two for the propellent tanks.
    # Each cylinder will have its own specified mass. The three smaller cylinders for the engines and tanks will be joined to the rocket body with rigid constraints.
    # The two tank cylinders will have variable mass as the propellent is used.

    # RP-1 density is 0.820
    # LOX density is 1.14, or maybe more like 1.25 if it is densified
    # First stage uses 297200 kg of LOX, 119700 kg of RP-1

    # Turn off damping, sleeping, and other bells and whistles
    self.f9BodyNP.node().set_linear_sleep_threshold(0)
    self.f9BodyNP.node().set_angular_sleep_threshold(0)
    self.f9BodyNP.node().set_linear_damping(0)
    self.f9BodyNP.node().set_angular_damping(0)

      # Make the rocket bounce off the ground for fun
    self.f9BodyNP.node().set_restitution(0.5)


    # Set initial position/velocity

    self.f9BodyNP.node().set_linear_velocity(LVector3f(0, 0, 9))
    self.f9BodyNP.node().set_angular_velocity(LVector3f(1, 1, 0))



  

    # Load models
    imagery.loadScenery(self.render.attachNewNode('sceneryNode'))

    rocketModelNodePath = self.f9BodyNP.attachNewNode('falconNode')
    falcon.loadFalcon(rocketModelNodePath, f9Radius, f9Height, f9COMoffset)

    # Lighting
    dlight = DirectionalLight('dlight')
    dlight.setColor(VBase4(0.8, 0.8, 0.8, 1))
    dlnp = self.render.attachNewNode(dlight)
    dlnp.setHpr(30, -60, 0)
    self.render.setLight(dlnp)

    alight = AmbientLight('alight')
    alight.setColor(VBase4(0.3, 0.3, 0.3, 1))
    alnp = self.render.attachNewNode(alight)
    self.render.setLight(alnp)

    self.render.setAntialias(AntialiasAttrib.MAuto)

    # Add the tick procedure to the main task manager.
    self.taskMgr.add(self.tick, "Tick")

    # Set up a second thread for the controller
    self.taskMgr.setupTaskChain('controller', numThreads = 1)
    self.taskMgr.add(self.runController, "Controller", taskChain='controller')

    # TODO: Clean shutdown
    self.accept('escape', sys.exit) 



    # Create a shared data dictionary to pass data to and from the controller
    # Remember to use locks when accessing shared data
    self.sharedData = { }

    # Lock for accessing shared data
    self.lock = threading.Lock()

  # Perform the physics here
  def tick(self, task):

    with self.lock:
      # TODO: Read controller output from shared data
      pass

    # Run Bullet simulation (where is globalClock defined? Well it works somehow...)
    dt = globalClock.getDt()
    self.world.doPhysics(dt)
    
    with self.lock:
      # TODO: Write current telemetry to shared data
      pass

    # Position camera to look at rocket
    
    self.camera.setPos(self.f9BodyNP.getPos() + Vec3(0, 120, 20))
    self.camera.lookAt(self.f9BodyNP)
    
    return Task.cont    # Execute the task again

  # Runs on a separate thread
  def runController(self, task):
    
    print 'Controller began on frame', task.frame
    
    with self.lock:
      # TODO: Read current telemetry from shared data
      pass

    # TODO: The CPU-intensive MHE/MPC will solve here
    time.sleep(1)

    with self.lock:
      # TODO: Write controller output to the shared data
      pass

    # Immediately begin this task again
    return Task.cont



app = MyApp()
app.run()
