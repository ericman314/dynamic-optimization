from model import getModel
import numpy as np
from time import sleep

class EstimatorController:
  def __init__(self):
    """Initialize the estimator (mhe) and controller (mpc)."""

    # ----- Estimator --------------------------
    self.mhe = getModel()

    self.mhe.options.IMODE = 5

    # Only estimating velocities from uncorrupted position data, so not many points needed here
    self.mhe.time = np.linspace(0, 4, 5)

    # Measured state variables and MVs
    self.mhe.x.FSTATUS = 1
    self.mhe.y.FSTATUS = 1
    self.mhe.z.FSTATUS = 1
    self.mhe.θ_x.FSTATUS = 1
    self.mhe.θ_y.FSTATUS = 1
    self.mhe.propMass.FSTATUS = 1
    self.mhe.Throttle.FSTATUS = 1
    self.mhe.EngineOn.FSTATUS = 1
    self.mhe.Gimbalx.FSTATUS = 1
    self.mhe.Gimbaly.FSTATUS = 1
    self.mhe.Gridx.FSTATUS = 1
    self.mhe.Gridy.FSTATUS = 1

    # Unmeasured, estimated state variables
    self.mhe.vx.STATUS = 1
    self.mhe.vy.STATUS = 1
    self.mhe.vz.STATUS = 1
    self.mhe.w_x.STATUS = 1
    self.mhe.w_y.STATUS = 1

    # ----- Controller --------------------------
    self.mpc = getModel()

  
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

    self.mhe.x.MEAS = x
    self.mhe.y.MEAS = x
    self.mhe.z.MEAS = x
    self.mhe.θ_x.MEAS = yaw * np.pi / 180
    self.mhe.θ_y.MEAS = pitch * np.pi / 180
    self.mhe.propMass.MEAS = prop

    self.mhe.solve()



  def getMHEVars(self):
    """Returns the estimated variables from the estimator.

    Returns
    -------
    array
      An array containing these variables in this order: x, y, z, vx, vy, vz, θ_x, θ_y, w_x, w_y, and propMass
    """

    return np.array([ self.mhe.x.value[-1],
                      self.mhe.y.value[-1],
                      self.mhe.z.value[-1],
                      self.mhe.vx.value[-1],
                      self.mhe.vy.value[-1],
                      self.mhe.vz.value[-1],
                      self.mhe.θ_x.value[-1],
                      self.mhe.θ_y.value[-1],
                      self.mhe.w_x.value[-1],
                      self.mhe.w_y.value[-1],
                      self.mhe.propMass.value[-1] ])


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
    A NumPy array representing the optimal values of MVs over the time horizon considered by the MPC. The first axis represents the time relative to the current time, and the second axis are the various MVs, in this order: Time, Throttle, EngineOn, GimbalX, GimbalY, GridX, GridY.
    """

    print( 'TODO: Run the MPC and return arrays of the MVs')

  