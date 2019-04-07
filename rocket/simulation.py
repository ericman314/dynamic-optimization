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
from runModelController import runController
import sys
import os.path
import math
from time import time, strftime, localtime, sleep
import numpy as np
import matplotlib.pyplot as plt

import imagery
import falcon

from math import pi, sin, cos


# Filename to read initial conditions from (don't include the .csv)
# initFilename = '500km-drop'
initFilename = '40km-10%prop-750mpsdown'

# Filename to read step tests from (don't include the .csv)
stepFilename = 'gimbal-rotate-high'

endTime = 15    # Set to 0 to run until hitting the ground

# Specify whether we are running the controller or the step tests
shouldRunController = False
shouldRunStepTests = True

# Disable individual forces (set to 0 to disable)
dragFactor = 1
liftFactor = 1
gridFactor = 1

# Miscellaneous configs for Panda3d
ConfigVariableDouble('default-far').setValue(20000000)
ConfigVariableInt('framebuffer-multisample').setValue(1)
ConfigVariableInt('multisamples').setValue(2)

# Global constants
g = 9.80665

# Helper functions
def dot(a, b):
  return a.x * b.x + a.y * b.y + a.z * b.z

def length(a):
  return math.sqrt(a.x**2 + a.y**2 + a.z**2)

def norm(a):
  anorm = Vec3(a)
  anorm.normalize()
  return anorm



class MyApp(ShowBase):
 
  def __init__(self):

    ShowBase.__init__(self)

    # Set up the Bullet physics engine
    self.world = BulletWorld()

    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(False)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = self.render.attachNewNode(debugNode)
    debugNP.show()
    
    self.world.setDebugNode(debugNP.node())

    # Make a ground for the rocket to land on
    self.groundNP = self.render.attachNewNode(BulletRigidBodyNode('Ground'))
    self.groundNP.setPos(0, 0, 0)
    self.groundNP.node().addShape(BulletPlaneShape(Vec3(0, 0, 1), 1))
    self.groundNP.node().set_restitution(0.5)

    self.world.attachRigidBody(self.groundNP.node())

    # Falcon 9 parameters
    self.f9ThrustSL = 7607000 / 9     # N, per engine
    self.f9ThrustVac = 8227000 / 9    # N, per engine
    self.f9MinimumThrottle = 0.57     # Minimum throttle capability
    self.f9PropMass = 417000          # mass of propellent when 100% full
    self.f9BodyMass = 27200           # mass of empty falcon 9 without propellent
    self.f9Radius = 1.8542     # Radius of F9 in meters
    self.f9Height = 47         # Height of F9 in meters
    self.f9COMoffset = -8            # Center of mass relative to center of body, in meters
    self.f9GridFinAuthority = 10      # Some arbitrary number to say how much lift the grid fins produce (oh, just realized we never use this anywhere)

    # Simulation variables    
    self.propLoad = 0.1 * self.f9PropMass    # Amount of full-tank propellent remaining
    self.gimbalX = 0      # Degrees
    self.gimbalY = 0    # Degrees
    self.throttle = 0.0  # Fraction between f9MinimumThrottle and 1.0
    self.engineOn = 0
    self.gridX = 0.0   # Degrees
    self.gridY = 0.0
    self.gridNegX = 0.0   # Degrees
    self.gridNegY = 0.0
    self.gridPosX = 0.0   # Degrees
    self.gridPosY = 0.0

    # Make the rocket
    self.f9BodyNP = self.render.attachNewNode(BulletRigidBodyNode('RocketBody'))
    
    self.f9BodyNP.node().addShape(BulletCylinderShape(self.f9Radius, self.f9Height, Z_up), TransformState.makePos(Vec3(0, 0, -self.f9COMoffset)))   # radius, height, axis
    # Rocket landing legs
    
    for i in range(4):
      leg = BulletCylinderShape(0.1, 10, X_up)
      self.f9BodyNP.node().addShape(leg, TransformState.makePosHpr(Vec3(6*math.cos(i*math.pi/2),6*math.sin(i*math.pi/2),-self.f9Height*0.5-self.f9COMoffset),Vec3(i*90,0,30)))
    

    # Set initial position/velocity
    # X (m), Y (m), Z (m), Roll (deg), Yaw (deg), Pitch (deg), Xdot (m/s), Ydot (m/s), Zdot (m/s), Prop (kg)
    initX, initY, initZ, initRoll, initYaw, initPitch, initXdot, initYdot, initZdot, self.propLoad = \
      np.loadtxt(os.path.join('initialConditions', initFilename + '.csv'), delimiter=',', unpack=True)

    self.f9BodyNP.node().setMass(self.f9BodyMass + self.propLoad)
    self.f9BodyNP.setPos(initX, initY, initZ)
    self.f9BodyNP.setHpr(initRoll, initPitch, initYaw)
    self.f9BodyNP.node().set_linear_velocity(LVector3f(initXdot, initYdot, initZdot))
    
    # Load step tests
    if shouldRunStepTests:
      self.stepTests = np.loadtxt(os.path.join('stepTests', stepFilename + '.csv'), delimiter=',')
      print (self.stepTests)


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



    self.npDragForce = LineNodePath(self.render, 'Drag', thickness=4, colorVec=VBase4(1, 0, 0, 1))
    self.npLiftForce = LineNodePath(self.render, 'Lift', thickness=4, colorVec=VBase4(0, 0.5, 1, 1))
    self.npGravForce = LineNodePath(self.render, 'Gravity', thickness=4, colorVec=VBase4(0.5, 0, 0.5, 1))
    self.npThrustForce = LineNodePath(self.render, 'Thrust', thickness=4, colorVec=VBase4(1, 0.5, 0, 1))
    self.npGridXposForce = LineNodePath(self.render, 'GridXpos', thickness=4, colorVec=VBase4(0, 0.5, 1, 1))
    self.npGridXnegForce = LineNodePath(self.render, 'GridXneg', thickness=4, colorVec=VBase4(0, 0.5, 1, 1))
    self.npGridYposForce = LineNodePath(self.render, 'GridYpos', thickness=4, colorVec=VBase4(0, 0.5, 1, 1))
    self.npGridYnegForce = LineNodePath(self.render, 'GridYneg', thickness=4, colorVec=VBase4(0, 0.5, 1, 1))
    self.npGridXposNormal = LineNodePath(self.render, 'GridXpos', thickness=4, colorVec=VBase4(1, 1, 0, 1))
    self.npGridXnegNormal = LineNodePath(self.render, 'GridXneg', thickness=4, colorVec=VBase4(1, 1, 0, 1))
    self.npGridYposNormal = LineNodePath(self.render, 'GridYpos', thickness=4, colorVec=VBase4(1, 1, 0, 1))
    self.npGridYnegNormal = LineNodePath(self.render, 'GridYneg', thickness=4, colorVec=VBase4(1, 1, 0, 1))

    # Load models
    imagery.loadScenery(self.render.attachNewNode('sceneryNode'))

    rocketModelNodePath = self.f9BodyNP.attachNewNode('falconNode')
    falcon.loadFalcon(rocketModelNodePath, self.f9Radius, self.f9Height, self.f9COMoffset)

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

    if shouldRunController:
      # Set up a second thread for the controller
      self.taskMgr.setupTaskChain('controller', numThreads = 1)
      self.taskMgr.add(self.runController, "Controller", taskChain='controller')

    self.accept('escape', self.userExit) 

    self.exitFunc = self.myExitFunc


    # Create a shared data dictionary to pass data to and from the controller
    # Initialize the shared data with null (zero) outputs
    # Remember to use locks when accessing shared data
    self.sharedData = { 'throttle': 0, 'gimbalX': 0, 'gimbalY': 0, 'gridX': 0, 'gridY': 0, 'engineOn': False }

    # Lock for accessing shared data
    self.lock = threading.Lock()

    # Initialize variables to compute derivatives
    self.f9VelLast = self.f9BodyNP.node().getLinearVelocity()
    quat = self.f9BodyNP.getTransform().getQuat()
    (f9Roll, f9Pitch, f9Yaw) = quat.getHpr()
    self.f9RollLast = f9Roll
    self.f9PitchLast = f9Pitch
    self.f9YawLast = f9Yaw

    # Initialize arrays to store detailed telemetry data for debugging, plotting, optimizing, etc.
    self.pltTime = np.zeros(0)
    self.pltX = np.zeros(0)
    self.pltY = np.zeros(0)
    self.pltZ = np.zeros(0)
    self.pltRoll = np.zeros(0)
    self.pltYaw = np.zeros(0)
    self.pltPitch = np.zeros(0)
    self.pltXdot = np.zeros(0)
    self.pltYdot = np.zeros(0)
    self.pltZdot = np.zeros(0)
    self.pltProp = np.zeros(0)
    self.pltThrottle = np.zeros(0)
    self.pltGimbalX = np.zeros(0)
    self.pltGimbalY = np.zeros(0)
    self.pltGridX = np.zeros(0)
    self.pltGridY = np.zeros(0)
    self.pltGeeAxial = np.zeros(0)
    self.pltGeeLateral = np.zeros(0)
    self.pltAOA = np.zeros(0)
    
    self.pltSaveInterval = 1.0    # seconds
    self.nextPltSaveTime = 0

    self.endTime = endTime    # 0 = five seconds after landing

    self.hasLandedOrCrashed = False


  # Perform the physics here
  def tick(self, task):

    if shouldRunController:
      with self.lock:
        # Read controller output from shared data
        self.throttle = self.sharedData['throttle']
        self.gimbalX = self.sharedData['gimbalX']
        self.gimbalY = self.sharedData['gimbalY']
        self.gridX = self.sharedData['gridX']
        self.gridY = self.sharedData['gridY']
        self.engineOn = self.sharedData['engineOn']

    if shouldRunStepTests:
      # Find the right time
      i = 0
      while i+1 < self.stepTests.shape[0] and self.stepTests[i+1][0] < task.time:
        i += 1
      # Time (sec), Throttle (0-1), EngineOn (0 or 1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg)
      self.throttle = self.stepTests[i, 1]
      self.engineOn = self.stepTests[i, 2]
      self.gimbalX = self.stepTests[i, 3]
      self.gimbalY = self.stepTests[i, 4]
      self.gridX = self.stepTests[i, 5]
      self.gridY = self.stepTests[i, 6]
      

    # Limit controller output
    self.throttle = max(min(self.throttle, 1.0), self.f9MinimumThrottle)
    self.gimbalX = max(min(self.gimbalX, 7), -7)
    self.gimbalY = max(min(self.gimbalY, 7), -7)
    self.gridX = max(min(self.gridX, 30), -30)
    self.gridY = max(min(self.gridY, 30), -30)

    # Run Bullet simulation (where is globalClock defined? Well it works somehow...)
    dt = globalClock.getDt()
    self.world.doPhysics(dt)

    f9Pos = self.f9BodyNP.getPos()
    f9Vel = self.f9BodyNP.node().getLinearVelocity()
    f9Acc = (f9Vel - self.f9VelLast) / dt
    self.f9VelLast = f9Vel

    quat = self.f9BodyNP.getTransform().getQuat()
    (f9Roll, f9Pitch, f9Yaw) = quat.getHpr()
    f9RollRate = (f9Roll - self.f9RollLast) / dt
    f9PitchRate = (f9Pitch - self.f9PitchLast) / dt
    f9YawRate = (f9Yaw - self.f9YawLast) / dt
    self.f9RollLast = f9Roll
    self.f9PitchLast = f9Pitch
    self.f9YawLast = f9Yaw

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
    
    # Simple roll controller
    rollAdjust = min(max(-f9RollRate - f9Roll * 0.3, -20), 20)
    self.gridXpos = -rollAdjust + self.gridX
    self.gridXneg = rollAdjust + self.gridX
    self.gridYpos = rollAdjust + self.gridY
    self.gridYneg = -rollAdjust + self.gridY


    # Calculate forces on the rocket (these are all in local coordinates)

    # Calculate our own gravity so we can display a vector
    fvGravWorld = Vec3(0, 0, -(self.f9BodyMass + self.propLoad) * g)
    fpGravWorld = Point3(0, 0, 0)

    atmosPress, atmosTemp, atmosRho = air_dens(f9Pos.z)


    # Calclate drag
    # Get relative air speed (TODO: Include wind)
    vRelAir = -f9Vel
    dynPress = 0.5 * atmosRho * dot(vRelAir, vRelAir)

    # Get angle of attack (deg)
    AOA = math.acos(min(max(dot(norm(vRelAir), norm(-f9ZWorld)), -1), 1))

    # Very simple and probably not correct drag coefficient
    Cd = 1.5

    # Very simple and probably not correct area
    dragArea = 10.8 + (174.3-10.8) * math.sin(AOA)

    dragForceMag = dynPress * Cd * dragArea
        
    fvDragWorld = norm(vRelAir) * dragForceMag * dragFactor
    fpDragWorld = quat.xform(Point3(0,0,-self.f9COMoffset))   # Center of vehicle

    # Calculate lift
    vLiftDirection = norm(vRelAir - vRelAir.project(f9ZWorld))
    if AOA > 0.5*math.pi:
      vLiftDirection = -vLiftDirection
    fvLiftWorld = vLiftDirection * (math.sin(AOA*2) * 174.3 * dynPress) * liftFactor
    fpLiftWorld = quat.xform(Point3(0,0,-self.f9COMoffset))   # Center of vehicle

    # Calculate lift for each of the grid fins

    # Calculate point of force application for each grid fin
    fpGridXposWorld = quat.xform(Point3(0, -self.f9Radius, 0.5 * self.f9Height - self.f9COMoffset))
    fpGridXnegWorld = quat.xform(Point3(0, self.f9Radius, 0.5 * self.f9Height - self.f9COMoffset))
    fpGridYposWorld = quat.xform(Point3(-self.f9Radius, 0, 0.5 * self.f9Height - self.f9COMoffset))
    fpGridYnegWorld = quat.xform(Point3(self.f9Radius, 0, 0.5 * self.f9Height - self.f9COMoffset))

    # Get transforms for each grid fin
    quatGridXpos = TransformState.makeHpr(Vec3(90, self.gridXpos, 0)).getQuat()
    quatGridYpos = TransformState.makeHpr(Vec3(180, self.gridYpos, 0)).getQuat()
    quatGridXneg = TransformState.makeHpr(Vec3(270, -self.gridXneg, 0)).getQuat()
    quatGridYneg = TransformState.makeHpr(Vec3(0, -self.gridYneg, 0)).getQuat()

    # Calculate each grid fin's normal vector in world coordinates (this is analagous to f9ZWorld for each fin)
    gridXposZWorld = quat.xform(quatGridXpos.xform(Vec3(0, 0, 1)))
    gridXnegZWorld = quat.xform(quatGridXneg.xform(Vec3(0, 0, 1)))
    gridYposZWorld = quat.xform(quatGridYpos.xform(Vec3(0, 0, 1)))
    gridYnegZWorld = quat.xform(quatGridYneg.xform(Vec3(0, 0, 1)))

    # Calculate angle of attack for each grid fin
    gridXposAOA = math.acos(min(max(dot(norm(vRelAir), norm(gridXposZWorld)), -1), 1))
    gridXnegAOA = math.acos(min(max(dot(norm(vRelAir), norm(gridXnegZWorld)), -1), 1))
    gridYposAOA = math.acos(min(max(dot(norm(vRelAir), norm(gridYposZWorld)), -1), 1))
    gridYnegAOA = math.acos(min(max(dot(norm(vRelAir), norm(gridYnegZWorld)), -1), 1))

    # Calculate lift force for each grid fin
    vLiftDirectionGridXpos = norm(vRelAir - vRelAir.project(gridXposZWorld))    
    vLiftDirectionGridXneg = norm(vRelAir - vRelAir.project(gridXnegZWorld))
    vLiftDirectionGridYpos = norm(vRelAir - vRelAir.project(gridYposZWorld))    
    vLiftDirectionGridYneg = norm(vRelAir - vRelAir.project(gridYnegZWorld))

    if gridXposAOA > 0.5*math.pi: vLiftDirectionGridXpos = -vLiftDirectionGridXpos
    if gridXnegAOA > 0.5*math.pi: vLiftDirectionGridXneg = -vLiftDirectionGridXneg
    if gridYposAOA > 0.5*math.pi: vLiftDirectionGridYpos = -vLiftDirectionGridYpos
    if gridYnegAOA > 0.5*math.pi: vLiftDirectionGridYneg = -vLiftDirectionGridYneg

    fvGridXposWorld = vLiftDirectionGridXpos * (math.sin(gridXposAOA*2) * 10 * dynPress) * gridFactor
    fvGridXnegWorld = vLiftDirectionGridXneg * (math.sin(gridXnegAOA*2) * 10 * dynPress) * gridFactor
    fvGridYposWorld = vLiftDirectionGridYpos * (math.sin(gridYposAOA*2) * 10 * dynPress) * gridFactor
    fvGridYnegWorld = vLiftDirectionGridYneg * (math.sin(gridYnegAOA*2) * 10 * dynPress) * gridFactor

    if not self.engineOn:
      self.throttle = 0

    # Update mass (each engine consumes 300kg/s of propellent at 100% throttle)
    self.propLoad -= 300 * dt * self.throttle
    
    if self.propLoad <= 0:
      self.propLoad = 0
      self.throttle = 0

    # Thrust

    thrust = self.throttle * (self.f9ThrustSL * atmosPress / 101325 + self.f9ThrustVac * (1 - atmosPress / 101325))
    fvThrustLocal = Vec3(0, 0, thrust)
    fpThrustLocal = Point3(0, 0, -self.f9COMoffset - self.f9Height * 0.5)
    quatGimbal = TransformState.makeHpr(Vec3(0, -self.gimbalY, self.gimbalX)).getQuat()
    fvThrustLocal = quatGimbal.xform(fvThrustLocal)

    # Important: In applyForce, the direction of the force is in world (not local) coordinates. The point of application is in world coordinates, but relative to the center of mass.
    # Transform fvDrag and fpDrag into world coordinates (but offset by the center of mass)
    

    fpThrustWorld = quat.xform(fpThrustLocal)
    fvThrustWorld = quat.xform(fvThrustLocal)
    

    # Apply forces (arguments to applyForce are in world coordinates, offset by the center of mass)
    self.f9BodyNP.node().applyForce(fvDragWorld, fpDragWorld)
    self.f9BodyNP.node().applyForce(fvLiftWorld, fpLiftWorld)
    self.f9BodyNP.node().applyForce(fvGravWorld, fpGravWorld)
    self.f9BodyNP.node().applyForce(fvThrustWorld, fpThrustWorld)
    self.f9BodyNP.node().applyForce(fvGridXposWorld, fpGridXposWorld)
    self.f9BodyNP.node().applyForce(fvGridXnegWorld, fpGridXnegWorld)
    self.f9BodyNP.node().applyForce(fvGridYposWorld, fpGridYposWorld)
    self.f9BodyNP.node().applyForce(fvGridYnegWorld, fpGridYnegWorld)



    self.f9BodyNP.node().setMass(self.f9BodyMass + self.propLoad)

    # Draw force arrows (arguments to drawArrow2d are in world coordinates)
    forceArrowScale = 2e-5

    self.npDragForce.reset()
    self.npDragForce.drawArrow2d(fpDragWorld + comWorld, fpDragWorld + fvDragWorld * forceArrowScale + comWorld, 45, 2)
    self.npDragForce.create()


    self.npLiftForce.reset()
    self.npLiftForce.drawArrow2d(fpDragWorld + comWorld, fpLiftWorld + fvLiftWorld * forceArrowScale + comWorld, 45, 2)
    self.npLiftForce.create()

    self.npGravForce.reset()
    self.npGravForce.drawArrow2d(fpGravWorld + comWorld, fpGravWorld + fvGravWorld * forceArrowScale + comWorld, 45, 2)
    self.npGravForce.create()

    # Draw thrust vector with tail at engine so we can see it
    self.npThrustForce.reset()
    self.npThrustForce.drawArrow2d((fpThrustWorld - fvThrustWorld * forceArrowScale) + comWorld, fpThrustWorld + comWorld, 45, 2)
    self.npThrustForce.create()

    self.npGridXposForce.reset()
    self.npGridXnegForce.reset()
    self.npGridYposForce.reset()
    self.npGridYnegForce.reset()
    self.npGridXposForce.drawArrow2d(fpGridXposWorld + comWorld, fpGridXposWorld + fvGridXposWorld * forceArrowScale + comWorld, 45, 2)
    self.npGridXnegForce.drawArrow2d(fpGridXnegWorld + comWorld, fpGridXnegWorld + fvGridXnegWorld * forceArrowScale + comWorld, 45, 2)
    self.npGridYposForce.drawArrow2d(fpGridYposWorld + comWorld, fpGridYposWorld + fvGridYposWorld * forceArrowScale + comWorld, 45, 2)
    self.npGridYnegForce.drawArrow2d(fpGridYnegWorld + comWorld, fpGridYnegWorld + fvGridYnegWorld * forceArrowScale + comWorld, 45, 2)
    self.npGridXposForce.create()
    self.npGridXnegForce.create()
    self.npGridYposForce.create()
    self.npGridYnegForce.create()

    # Draw grid fin normals
    self.npGridXposNormal.reset()
    self.npGridXnegNormal.reset()
    self.npGridYposNormal.reset()
    self.npGridYnegNormal.reset()
    self.npGridXposNormal.drawArrow2d(fpGridXposWorld + comWorld, fpGridXposWorld + gridXposZWorld * 10 + comWorld, 45, 2)
    self.npGridXnegNormal.drawArrow2d(fpGridXnegWorld + comWorld, fpGridXnegWorld + gridXnegZWorld * 10 + comWorld, 45, 2)
    self.npGridYposNormal.drawArrow2d(fpGridYposWorld + comWorld, fpGridYposWorld + gridYposZWorld * 10 + comWorld, 45, 2)
    self.npGridYnegNormal.drawArrow2d(fpGridYnegWorld + comWorld, fpGridYnegWorld + gridYnegZWorld * 10 + comWorld, 45, 2)
    self.npGridXposNormal.create()
    self.npGridXnegNormal.create()
    self.npGridYposNormal.create()
    self.npGridYnegNormal.create()
    
    
    # Draw onscreen text
    osdText = []
    osdText.append('Time (s): {:.1f}'.format(task.time))
    osdText.append('Speed (m/s): {:.1f}'.format(math.sqrt(f9Vel.x**2 + f9Vel.y**2 + f9Vel.z**2)))
    osdText.append('Altitude (km): {:.2f}'.format(f9Pos.z*1e-3))
    osdText.append('Downrange (km): {:.2f}'.format(math.sqrt(f9Pos.x**2 + f9Pos.y**2)*1e-3))
    osdText.append('Gee (axial): {:0.2f}'.format(geeAxial))
    osdText.append('Gee (lateral): {:0.2f}'.format(geeLateral))
    osdText.append('AOA (deg): {:.2f}'.format(AOA / math.pi * 180))
    osdText.append('Ambient pressure (kPa): {:.2f}'.format(atmosPress/1000))
    osdText.append('Dynamic pressure (kPa): {:.2f}'.format(dynPress/1000))
    osdText.append('')
    osdText.append('Throttle (%): {:.0f}'.format(self.throttle * 100))
    osdText.append('Thrust (kN): {:.0f}'.format(thrust/1000))
    osdText.append('Propellent (%): {:.1f}'.format(self.propLoad / self.f9PropMass * 100))
    osdText.append('Gimbal (deg): {:5.2f} x {:5.2f}'.format(self.gimbalX, self.gimbalY))
    osdText.append('Grid fins (deg): {:5.2f} x {:5.2f}'.format(self.gridX, self.gridY))

    self.npTelemetryFeed.setText('\n'.join(osdText))

    if self.endTime == 0 and f9Pos.z < 20:
      print ('Rocket has landed or tipped over. Ending the simulation in 5 seconds.')
      self.hasLandedOrCrashed = True
      self.endTime = task.time + 5

    if self.endTime > 0 and task.time > self.endTime:
      self.userExit()


    # Save telemetry to arrays
    if task.time > self.nextPltSaveTime and not self.hasLandedOrCrashed:
      self.nextPltSaveTime += self.pltSaveInterval
      self.pltTime =       np.append(self.pltTime,       [task.time])
      self.pltX =          np.append(self.pltX,          [f9Pos.x])
      self.pltY =          np.append(self.pltY,          [f9Pos.y])
      self.pltZ =          np.append(self.pltZ,          [f9Pos.z])
      self.pltRoll =       np.append(self.pltRoll,       [f9Roll])
      self.pltYaw =        np.append(self.pltYaw,        [f9Yaw])
      self.pltPitch =      np.append(self.pltPitch,      [f9Pitch])
      self.pltXdot =       np.append(self.pltXdot,       [f9Vel.x])
      self.pltYdot =       np.append(self.pltYdot,       [f9Vel.y])
      self.pltZdot =       np.append(self.pltZdot,       [f9Vel.z])
      self.pltProp =       np.append(self.pltProp,       [self.propLoad])
      self.pltThrottle =   np.append(self.pltThrottle,   [self.throttle])
      self.pltGimbalX =    np.append(self.pltGimbalX,    [self.gimbalX])
      self.pltGimbalY =    np.append(self.pltGimbalY,    [self.gimbalY])
      self.pltGridX =      np.append(self.pltGridX,      [self.gridX])
      self.pltGridY =      np.append(self.pltGridY,      [self.gridY])
      self.pltGeeAxial =   np.append(self.pltGeeAxial,   [geeAxial])
      self.pltGeeLateral = np.append(self.pltGeeLateral, [geeLateral])
      self.pltAOA =        np.append(self.pltAOA,        [AOA/math.pi*180])


    with self.lock:
      # Write current telemetry to shared data
      self.sharedData['x'] = f9Pos.x
      self.sharedData['y'] = f9Pos.y
      self.sharedData['z'] = f9Pos.z
      self.sharedData['yaw'] = f9Yaw
      self.sharedData['pitch'] = f9Pitch
      self.sharedData['prop'] = self.propLoad

    # Position camera to look at rocket
    
    self.camera.setPos(self.f9BodyNP.getPos() + Vec3(-150, 0, 80))
    self.camera.lookAt(self.f9BodyNP)
    
    return Task.cont    # Execute the task again

  # Runs on a separate thread
  def runController(self, task):
    
    print ('Controller began on frame', task.frame)
    
    with self.lock:
      x = self.sharedData['x']
      y = self.sharedData['y']
      z = self.sharedData['z']
      yaw = self.sharedData['yaw']
      pitch = self.sharedData['pitch']
      prop = self.sharedData['prop']


    result = runController(x=x, y=y, z=z, yaw=yaw, pitch=pitch, prop=prop, time=task.time)

    with self.lock:
      self.sharedData['throttle'] = result['throttle']
      self.sharedData['gimbalX'] = result['gimbalX']
      self.sharedData['gimbalY'] = result['gimbalY']
      self.sharedData['gridX'] = result['gridX']
      self.sharedData['gridY'] = result['gridY']
      self.sharedData['engineOn'] = result['engineOn']
      self.sharedData['time'] = task.time

    print (self.sharedData)

    # Immediately begin this task again
    return Task.cont

  def myExitFunc(self):
    print ('Shutting down')

    dirName = 'simulationData'
    outputFilename = initFilename + '_' + stepFilename + '.csv'
    fnData = os.path.join(dirName, outputFilename)
    print ('Writing data to ' + fnData)

    data = np.vstack((self.pltTime, self.pltX, self.pltY, self.pltZ, self.pltRoll, self.pltYaw, self.pltPitch, self.pltXdot, self.pltYdot, self.pltZdot, self.pltProp, self.pltThrottle, self.pltGimbalX, self.pltGimbalY, self.pltGridX, self.pltGridY, self.pltGeeAxial, self.pltGeeLateral, self.pltAOA)).transpose()
    top = 'Time (sec), X (m), Y (m), Z (m), Roll (deg), Yaw (deg), Pitch (deg), Xdot (m/s), Ydot (m/s), Zdot (m/s), Prop (kg), Throttle (0-1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg), GeeAxial (g), GeeLateral (g), AOA (deg)'
    np.savetxt(fnData, data, fmt='%.2f', delimiter=', ', header=top)

    print ('Generating a few interesting plots')

    plt.figure(figsize=(11,8))
    plt.subplot(2, 1, 1)
    plt.plot(self.pltTime, self.pltX)
    plt.plot(self.pltTime, self.pltY)
    plt.plot(self.pltTime, self.pltZ)
    plt.subplot(2, 1, 2)
    plt.plot(self.pltTime, self.pltXdot)
    plt.plot(self.pltTime, self.pltYdot)
    plt.plot(self.pltTime, self.pltZdot)
    
    plt.show()



app = MyApp()
app.run()
