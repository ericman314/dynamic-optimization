from model import getModel
from time import sleep

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
    """Updates the MPC with current estimated variables.

    Parameters
    ----------
    v : array
      An array containing these variables in this order: x, y, z, vx, vy, vz, θ_x, θ_y, w_x, w_y, and propMass
    """

    print(' TODO: Update the MPC with the given variables' )


  def runMPC(self):
    """Runs the MPC, calculating and returning the values representing the optimal values of the manipulated variables.
    
    Returns
    -------
    A NumPy array representing the optimal values of MVs over the time horizon considered by the MPC. The first axis represents the time relative to the current time, and the second axis are the various MVs, in this order: Time, Throttle, EngineOn, Yaw, Pitch.
    """

    print( 'TODO: Run the MPC and return arrays of the MVs')

  