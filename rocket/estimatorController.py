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
    stepTime = 5 # seconds per time step
    timeHorizon = 60 # time horizon
    nt = int(timeHorizon/stepTime)+1 #number of time points for each cycle

    m = self.mpc
    m.time = np.linspace(0,timeHorizon,nt)


      # Position
    m.x.STATUS = 0
    m.x.FSTATUS = 0  # Receive measurement from the simulation. Not yet!!

    m.y.STATUS = 0
    m.y.FSTATUS = 0  # Receive measurement from simulation. Not yet

    m.z.STATUS = 1
    m.z.FSTATUS = 1  # Receive measurement from simulation. Not sure how to do it
    m.z.SP = 0.0 # setpoints could be updated fron the simluation data to change dynamically.
    m.z.TAU = 60 # time constant for position. Needs to be adjusted also.
    m.z.WSP = 100

    # Velocity
    m.vx.STATUS = 0
    m.vy.STATUS = 0
    m.vz.STATUS = 1  # 
    m.vz.FSTATUS = 1  # Receive measurement from simulation
    m.vz.SP = 0.0 # setpoint for vz
    m.vz.TAU = 60 # time constant

    # Adjustable parameters

    # m.liftAuthority.FSTATUS = 1 # Receive new values from the estimator. Not yet
    # m.dragAuthority.FSTATUS = 1 # Receive new values from the estimator. Not yet
    # Ifactorempirical.FSTATUS = 1 # Receive new values from the estimator. Not yet
    m.liftAuthority.FSTATUS = 0 # Receive new values from the estimator. Not yet
    m.dragAuthority.FSTATUS = 0 # Receive new values from the estimator. Not yet
    ## Manipulated variables for controller
    m.Throttle.STATUS = 1 # Adjust for controller
    m.Throttle.FSTATUS = 0 # Do not receive measurements
    m.EngineOn.STATUS = 1
    m.EngineOn.FSTATUS = 0
    m.Yaw.STATUS = 1
    m.Yaw.FSTATUS = 0
    m.Pitch.STATUS = 1
    m.Pitch.FSTATUS = 0

    m.options.CV_TYPE = 2
    m.options.NODES = 3
    m.options.SOLVER = 1
    m.options.IMODE = 6
    m.options.MAX_ITER = 500

    # Since there's no estimator active yet, the estimated parameters will be the same


    """Updates the MPC with current estimated variables.

    Parameters
    ----------
    v : array
      An array containing these variables in this order: x, y, z, vx, vy, vz, θ_x, θ_y, w_x, w_y, and propMass
    """

    # print(' TODO: Update the MPC with the given variables' )


  def runMPC(self):
    m=self.mpc
    m.solve()

    ## It's missing to communicate eith the estimator and pass values


    """Runs the MPC, calculating and returning the values representing the optimal values of the manipulated variables.
    
    Returns
    -------
    A NumPy array representing the optimal values of MVs over the time horizon considered by the MPC. The first axis represents the time relative to the current time, and the second axis are the various MVs, in this order: Time, Throttle, EngineOn, Yaw, Pitch.
    """

    print( 'TODO: Run the MPC and return arrays of the MVs')

  