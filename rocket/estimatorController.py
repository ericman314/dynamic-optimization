from model import getModel
# from time import sleep
import numpy as np
import time

class EstimatorController:
  def __init__(self):
    """Initialize the estimator (mhe) and controller (mpc)."""

    self.mhe = getModel(name='mhe')
    # Set mode, time horizon, tuning params, options, etc. for the MHE

    self.mpc = getModel(name='mpc')
    # Set mode, time horizon, tuning params, options, etc. for the MPC


    # stepTime = 5 # seconds per time step
    m = self.mpc

      # Position
    m.x.STATUS = 0
    m.x.FSTATUS = 1  # Receive measurement from the simulation. Not yet!!

    m.y.STATUS = 0
    m.y.FSTATUS = 1  # Receive measurement from simulation. Not yet

    # m.z.STATUS = 1
    m.z.FSTATUS = 1  # Receive measurement from simulation. Not sure how to do it
    # m.z.SP = 0.0 # setpoints could be updated fron the simluation data to change dynamically.
    # m.z.TAU = 60 # time constant for position. Needs to be adjusted also.
    # m.z.WSP = 100

    # Velocity
    m.vx.STATUS = 0
    m.vy.STATUS = 0
    m.vz.STATUS = 0  # 
    m.vx.FSTATUS = 1
    m.vy.FSTATUS = 1
    m.vz.FSTATUS = 1  # Receive measurement from simulation
    # m.vz.SP = 0.0 # setpoint for vz
    # m.vz.TAU = 60 # time constant

    # Adjustable parameters

    # m.liftAuthority.FSTATUS = 1 # Receive new values from the estimator. Not yet
    # m.dragAuthority.FSTATUS = 1 # Receive new values from the estimator. Not yet
    # Ifactorempirical.FSTATUS = 1 # Receive new values from the estimator. Not yet
    m.liftAuthority.FSTATUS = 0 # Receive new values from the estimator. Not yet
    m.dragAuthority.FSTATUS = 0 # Receive new values from the estimator. Not yet
    ## Manipulated variables for controller
    m.Throttle.FSTATUS = 0 # Do not receive measurements
    # m.EngineOn.STATUS = 1
    m.EngineOn.FSTATUS = 0
    m.Yaw.FSTATUS = 0
    m.Pitch.FSTATUS = 0

    m.Throttle.STATUS = 1 # Adjust for controller
    m.Yaw.STATUS = 1
    m.Pitch.STATUS = 1

    m.options.CV_TYPE = 2
    m.options.NODES = 3
    m.options.SOLVER = 1
    m.options.IMODE = 6
    m.options.MAX_ITER = 500

    # Since there's no estimator active yet, the estimated parameters will be the same
    
    # This will change each time the controller solves
    m.finalMask = m.Param()

    m.Obj( (m.vx**2 + m.x**2) * m.finalMask  )
    m.Obj( (m.vy**2 + m.y**2) * m.finalMask  )
    # m.Obj(m.x**2 + m.y**2)
    m.Obj( (m.vz**2 + (m.z-25)**2) * m.finalMask )
    m.Obj( (m.sqrt(m.z**2)-m.z)**2 )
    

    self.hasAddedAdditionalZObjective = False
    self.isInTerminalGuidance = False
    
  
  def runMHE(self, time, x, y, z, yaw, pitch, prop):
    """Updates the estimator with current measured values from the physical process or simulation,
    and runs the estimator to calculate certain unmeasured variables of the model. 

    The variables which are estimated are:
    x, y, z
    vx, vy, vz
    θ_x, θ_y
    w_x, w_y
    propMass

    Parameters
    ----------
    time : float
      The current simulation time.
    x, y, z : float
      The current measured x, y, and z coordinates of the rocket.
    yaw, pitch : float
      The current measured yaw and pitch, in degrees.
    prop : float
      The current measured propellent remaining, in kg.
    """    

    print( 'TODO: Run the estimator' )


  def getMHEVars(self):
    """Returns the estimated variables from the estimator.

    Returns
    -------
    array
      An array containing these variables in this order: x, y, z, vx, vy, vz, θ_x, θ_y, w_x, w_y, and propMass
    """

    print( 'TODO: Return the estimated variables from the mhe' )


  def setMPCVars(self, v):
    
    """Updates the MPC with current estimated variables.

    Parameters
    ----------
    v : array
      An array containing these variables in this order: simTime, x, y, z, vx, vy, vz, and propMass
    """

    simTime = v[0]

    # Assume the rocket will land at t = 70 seconds.
    timeHorizon = 78 - simTime # time horizon
    if timeHorizon < 0:
      raise ValueError("Controller is finished, gravity shall rule forever")
    if timeHorizon < 35:
      stepTime = 1
    else:
      stepTime = 2

    nt = int(timeHorizon/stepTime)+1 #number of time points for each cycle

    m = self.mpc
    m.time = np.linspace(0,timeHorizon,nt)

    # If the rocket is close to the ground, disable Yaw and Pitch. We want it to land upright, and it's too late to make adjustments anyway.
    if v[3] < 200:
      m.Yaw.STATUS = 0
      m.Pitch.STATUS = 0
      m.Yaw.VALUE = 0
      m.Pitch.VALUE = 0

    # Initialize the MPC
    m.x.VALUE = v[1]
    m.y.VALUE = v[2]
    m.z.VALUE = v[3]
    m.vx.VALUE = v[4]
    m.vy.VALUE = v[5]
    m.vz.VALUE = v[6]
    m.propMass.VALUE = v[7]

    _finalMask = np.zeros(m.time.size)
    _finalMask[-1] = 1
    m.finalMask.VALUE = _finalMask

    # Assume the engine turns on at t = 46 seconds
    engineOnTime = 40
    _engineOn = map(lambda x: 1 if x + simTime > engineOnTime else 0, m.time)
    m.EngineOn.VALUE = np.array(list(_engineOn))

    # If close to the ground, increase the weight of the z objective term.
    if simTime > 55 and not self.hasAddedAdditionalZObjective:
      self.hasAddedAdditionalZObjective = True
      m.Obj( (m.vz**2 * 100 + (m.z-25)**2 * 10) * m.finalMask)
      
    # If very close to the ground, just land it, don't try to hit the target
    if simTime > 68 and not self.isInTerminalGuidance:
      self.isInTerminalGuidance = True
      # m.Obj( -(m.x**2) * m.finalMask  )
      # m.Obj( -(m.y**2) * m.finalMask  )


  def runMPC(self, firstRun=False):
    m=self.mpc

    if firstRun:
      m.options.MAX_TIME = 60

      m.options.COLDSTART = 1

      m.options.RTOL = 0.1
      m.options.OTOL = 0.1
      

      m.solve()


    else:
      m.options.MAX_TIME = 5
      m.options.RTOL = 1e-3
      m.options.OTOL = 1e-3
    

    m.solve(disp=firstRun)

    print ('Controller objective: {:10.4f} Final position (x, y, z) (vx, vy, vz): ({:10.4f}, {:10.4f}, {:10.4f}) ({:10.4f}, {:10.4f}, {:10.4f})'.format(m.options.OBJFCNVAL, m.x.VALUE[-1], m.y.VALUE[-1], m.z.VALUE[-1], m.vx.VALUE[-1], m.vy.VALUE[-1], m.vz.VALUE[-1]))

    retVal = np.array((
      m.time,
      m.Throttle,
      m.EngineOn,
      m.Yaw,
      m.Pitch
    ))

    # print (retVal)
    return retVal

    """Runs the MPC, calculating and returning the values representing the optimal values of the manipulated variables.
    
    Returns
    -------
    A NumPy array representing the optimal values of MVs over the time horizon considered by the MPC. The first axis represents the time relative to the current time, and the second axis are the various MVs, in this order: Time, Throttle, EngineOn, Yaw, Pitch.
    """

    print( 'TODO: Run the MPC and return arrays of the MVs')

  