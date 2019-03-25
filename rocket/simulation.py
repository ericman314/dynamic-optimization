from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from direct.stdpy import threading
from direct.directtools.DirectGeometry import LineNodePath
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Point3, Vec3, ConfigVariableDouble, ConfigVariableInt, DirectionalLight, AmbientLight, VBase4, LVector3f, AntialiasAttrib, TransformState, TextNode
from panda3d.bullet import BulletWorld, BulletCylinderShape, BulletPlaneShape, BulletRigidBodyNode, X_up, Y_up, Z_up, BulletGenericConstraint, BulletDebugNode
from Environ_dependancies import air_dens, Fg
import time
import sys
import math

import imagery
import falcon

from math import pi, sin, cos
 
ConfigVariableDouble('default-far').setValue(20000000)
ConfigVariableInt('framebuffer-multisample').setValue(1)
ConfigVariableInt('multisamples').setValue(2)

g = 9.80665

def dot(a, b):
  return a.x * b.x + a.y * b.y + a.z * b.z

def length(a):
  return math.sqrt(a.x**2 + a.y**2 + a.z**2)

class MyApp(ShowBase):
 
  def __init__(self):

    ShowBase.__init__(self)

    # Set up the Bullet physics engine
    self.world = BulletWorld()
    # self.world.setGravity(Vec3(0, 0, -g))

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
    self.f9ThrustSL = 7607000 / 9     # N, per engine
    self.f9ThrustVac = 8227000 / 9    # N, per engine
    self.f9MinimumThrottle = 0.57     # Minimum throttle capability
    self.f9PropMass = 417000          # mass of propellent when 100% full
    self.f9BodyMass = 27200           # mass of empty falcon 9 without propellent
    f9Radius = 1.8542     # Radius of F9 in meters
    self.f9Height = 47         # Height of F9 in meters
    self.f9COMoffset = -8            # Center of mass relative to center of body, in meters
    
    # Initial Conditions
    initX = 0
    initY = 0
    initZ = 20

    # Simulation variables    
    self.propLoad = 0.1 * self.f9PropMass    # Amount of full-tank propellent remaining
    self.gimbalX = 0      # Degrees
    self.gimbalY = 0    # Degrees
    self.throttle = 1.0  # Fraction between f9MinimumThrottle and 1.0

    # Make the rocket
    self.f9BodyNP = self.render.attachNewNode(BulletRigidBodyNode('RocketBody'))
    
    self.f9BodyNP.node().addShape(BulletCylinderShape(f9Radius, self.f9Height, Z_up), TransformState.makePos(Vec3(0, 0, -self.f9COMoffset)))   # radius, height, axis
    # Rocket landing legs
    
    for i in range(4):
      leg = BulletCylinderShape(0.1, 10, X_up)
      self.f9BodyNP.node().addShape(leg, TransformState.makePosHpr(Vec3(6*math.cos(i*math.pi/2),6*math.sin(i*math.pi/2),-self.f9Height*0.5-self.f9COMoffset),Vec3(i*90,0,30)))
    

    
    self.f9BodyNP.node().setMass(self.f9BodyMass + self.propLoad)
    self.f9BodyNP.setPos(initX, initY, initZ)
    self.f9BodyNP.setHpr(0, 0, 0)


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

    self.f9BodyNP.node().set_linear_velocity(LVector3f(0, 0, 0))
    self.f9BodyNP.node().set_angular_velocity(LVector3f(0, 0, 1))
    


    self.npDragForce = LineNodePath(self.render, 'Drag', thickness=4, colorVec=VBase4(1, 0, 0, 1))
    self.npGravForce = LineNodePath(self.render, 'Gravity', thickness=4, colorVec=VBase4(0.5, 0, 0.5, 1))
    self.npThrustForce = LineNodePath(self.render, 'Thrust', thickness=4, colorVec=VBase4(1, 0.5, 0, 1))
  
    # Load models
    imagery.loadScenery(self.render.attachNewNode('sceneryNode'))

    rocketModelNodePath = self.f9BodyNP.attachNewNode('falconNode')
    falcon.loadFalcon(rocketModelNodePath, f9Radius, self.f9Height, self.f9COMoffset)

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

    # Onscreen text
    font = loader.loadFont('Lato-Regular.ttf')   
    self.npTelemetryFeed = OnscreenText(text = 'Waiting for downlink...', parent=base.a2dTopLeft, font=font, align=TextNode.ALeft, scale = 0.06, pos=(0.05, -0.1), fg=(1,1,1,1), shadow=(0,0,0,0.5))

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

    self.f9VelLast = self.f9BodyNP.node().getLinearVelocity()

  # Perform the physics here
  def tick(self, task):

    with self.lock:
      # TODO: Read controller output from shared data
      pass

    # Run Bullet simulation (where is globalClock defined? Well it works somehow...)
    dt = globalClock.getDt()
    self.world.doPhysics(dt)

    f9Pos = self.f9BodyNP.getPos()
    f9Vel = self.f9BodyNP.node().getLinearVelocity()
    f9Acc = (f9Vel - self.f9VelLast) / dt
    self.f9VelLast = f9Vel

    quat = self.f9BodyNP.getTransform().getQuat()
    comWorld = self.f9BodyNP.getPos()

    # Compute on-axis and lateral gee-forces
    f9XWorld = quat.xform(Vec3(1, 0, 0))
    f9YWorld = quat.xform(Vec3(0, 1, 0))
    f9ZWorld = quat.xform(Vec3(0, 0, 1))

    # Add gravity to the acceleration "felt"
    f9AccFelt = f9Acc + Vec3(0, 0, g)
    geeAxial = dot(f9AccFelt, f9ZWorld) / g
    geeLateral = f9AccFelt.project(f9XWorld) + f9AccFelt.project(f9YWorld)
    geeLateral = length(geeLateral) / g
    


    
    self.gimbalX = 0 #math.sin(task.time*2)
    self.gimbalY = 0 #math.cos(task.time*2)

    # Calculate forces on the rocket (these are all in local coordinates)

    # Calculate our own gravity so we can display a vector
    fvGravWorld = Vec3(0, 0, -(self.f9BodyMass + self.propLoad) * g)
    fpGravWorld = Point3(0, 0, 0)

    # Calclate drag

    # Just use a very simple expression for now, no angle of attack or anything
    
    fvDragWorld = Vec3(0,0,0)
    fpDragWorld = Point3(0,0,-self.f9COMoffset)    # Center of vehicle


    # Update mass (each engine consumes 300kg/s of propellent at 100% throttle)
    self.propLoad -= 300 * dt * self.throttle
    
    if self.propLoad <= 0:
      self.propLoad = 0
      self.throttle = 0

    # Thrust

    atmosPress, atmosTemp, atmosRho = air_dens(f9Pos.z)


    thrust = self.throttle * (self.f9ThrustSL * atmosPress / 101325 + self.f9ThrustVac * (1 - atmosPress / 101325))
    fvThrustLocal = Vec3(0, 0, thrust)
    fpThrustLocal = Point3(0, 0, -self.f9COMoffset - self.f9Height * 0.5)
    quatGimbal = TransformState.makeHpr(Vec3(0, self.gimbalY, self.gimbalX)).getQuat()
    fvThrustLocal = quatGimbal.xform(fvThrustLocal)

    # Important: In applyForce, the direction of the force is in world (not local) coordinates. The point of application is in world coordinates, but relative to the center of mass.
    # Transform fvDrag and fpDrag into world coordinates (but offset by the center of mass)
    

    # fpDragWorld = quat.xform(fpDragLocal)
    # fvDragWorld = quat.xform(fvDragLocal)

    fpThrustWorld = quat.xform(fpThrustLocal)
    fvThrustWorld = quat.xform(fvThrustLocal)
    

    # Apply forces (arguments to applyForce are in world coordinates, offset by the center of mass)
    self.f9BodyNP.node().applyForce(fvDragWorld, fpDragWorld)
    self.f9BodyNP.node().applyForce(fvGravWorld, fpGravWorld)
    self.f9BodyNP.node().applyForce(fvThrustWorld, fpThrustWorld)



    self.f9BodyNP.node().setMass(self.f9BodyMass + self.propLoad)

    # Draw force arrows (arguments to drawArrow2d are in world coordinates)
    forceArrowScale = 2e-5

    self.npDragForce.reset()
    self.npDragForce.drawArrow2d(fpDragWorld + comWorld, fpDragWorld + fvDragWorld * forceArrowScale + comWorld, 45, 2)
    self.npDragForce.create()

    self.npGravForce.reset()
    self.npGravForce.drawArrow2d(fpGravWorld + comWorld, fpGravWorld + fvGravWorld * forceArrowScale + comWorld, 45, 2)
    self.npGravForce.create()

    # Draw thrust vector with tail at engine so we can see it
    self.npThrustForce.reset()
    self.npThrustForce.drawArrow2d((fpThrustWorld - fvThrustWorld * forceArrowScale) + comWorld, fpThrustWorld + comWorld, 45, 2)
    self.npThrustForce.create()
    
    # Draw onscreen text
    osdText = []
    osdText.append('Speed (m/s): {:.1f}'.format(math.sqrt(f9Vel.x**2 + f9Vel.y**2 + f9Vel.z**2)))
    osdText.append('Altitude (km): {:.2f}'.format(f9Pos.z*1e-3))
    osdText.append('Downrange (km): {:.2f}'.format(math.sqrt(f9Pos.x**2 + f9Pos.y**2)*1e-3))
    osdText.append('Gee (axial): {:0.2f}'.format(geeAxial))
    osdText.append('Gee (lateral): {:0.2f}'.format(geeLateral))
    osdText.append('Ambient pressure (kPa): {:.2f}'.format(atmosPress/1000))
    osdText.append('')
    osdText.append('Throttle (%): {:.0f}'.format(self.throttle * 100))
    osdText.append('Thrust (kN): {:.0f}'.format(thrust/1000))
    osdText.append('Propellent (%): {:.1f}'.format(self.propLoad / self.f9PropMass * 100))
    osdText.append('Gimbal (deg): {:5.2f} x {:5.2f}'.format(self.gimbalX, self.gimbalY))

    self.npTelemetryFeed.setText('\n'.join(osdText))


    # Simple controller
    self.throttle = (1000 - f9Pos.z - f9Vel.z*10) / 100
    if self.throttle > 1: self.throttle = 1
    if self.throttle < 0: self.throttle = 0


    with self.lock:
      # TODO: Write current telemetry to shared data
      pass

    # Position camera to look at rocket
    
    self.camera.setPos(self.f9BodyNP.getPos() + Vec3(0, -180, 20))
    self.camera.lookAt(self.f9BodyNP)
    
    return Task.cont    # Execute the task again

  # Runs on a separate thread
  def runController(self, task):
    
    print ('Controller began on frame', task.frame)
    
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
