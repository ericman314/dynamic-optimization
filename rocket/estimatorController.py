from model import getModel
import numpy as np
from time import sleep

class EstimatorController:
  def __init__(self):
    """Initialize the estimator (mhe) and controller (mpc)."""
    """
    # ----- Estimator --------------------------
    self.mhe = getModel(name='mhe')

    self.mhe.options.MAX_TIME = 1

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
    # self.mhe.EngineOn.FSTATUS = 1
    self.mhe.Gimbalx.FSTATUS = 1
    self.mhe.Gimbaly.FSTATUS = 1
    self.mhe.Gridx.FSTATUS = 1
    self.mhe.Gridy.FSTATUS = 1

    # This seems to work
    # self.mhe.options.ICD_CALC = 1

    self.mhe.options.COLDSTART = 2
    """
    # ----- Controller --------------------------
    self.mpc = getModel(name='mpc')

    self.mpc.options.IMODE = 6
    # self.mpc.TIME = np.linspace(0, 60, 61)

    self.mpc.x.FSTATUS = 1
    self.mpc.y.FSTATUS = 1
    self.mpc.z.FSTATUS = 1
    self.mpc.vx.FSTATUS = 1
    self.mpc.vy.FSTATUS = 1
    self.mpc.vz.FSTATUS = 1
    self.mpc.θ_x.FSTATUS = 1
    self.mpc.θ_y.FSTATUS = 1
    self.mpc.w_x.FSTATUS = 1
    self.mpc.w_y.FSTATUS = 1
    self.mpc.propMass.FSTATUS = 1

    
    
  
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
    self.mhe.y.MEAS = y
    self.mhe.z.MEAS = z
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

    return np.array([ self.mhe.x.MODEL,
                      self.mhe.y.MODEL,
                      self.mhe.z.MODEL,
                      self.mhe.vx.MODEL,
                      self.mhe.vy.MODEL,
                      self.mhe.vz.MODEL,
                      self.mhe.θ_x.MODEL,
                      self.mhe.θ_y.MODEL,
                      self.mhe.w_x.MODEL,
                      self.mhe.w_y.MODEL,
                      self.mhe.propMass.MODEL ])


  def setMPCVars(self, v):
    """Updates the MPC with current estimated variables.

    Parameters
    ----------
    v : array
      An array containing these variables in this order: x, y, z, vx, vy, vz, θ_x, θ_y, w_x, w_y, and propMass
    """

    self.mpc.x.VALUE = v[0]
    self.mpc.y.VALUE = v[1]
    self.mpc.z.VALUE = v[2]
    self.mpc.vx.VALUE = v[3]
    self.mpc.vy.VALUE = v[4]
    self.mpc.vz.VALUE = v[5]
    self.mpc.θ_x.VALUE = v[6]
    self.mpc.θ_y.VALUE = v[7]
    self.mpc.w_x.VALUE = v[8]
    self.mpc.w_y.VALUE = v[9]
    self.mpc.propMass.VALUE = v[10]

    

  def runMPC(self):
    """Runs the MPC, calculating and returning the values representing the optimal values of the manipulated variables.
    
    Returns
    -------
    A NumPy array representing the optimal values of MVs over the time horizon considered by the MPC. The first axis represents the time relative to the current time, and the second axis are the various MVs, in this order: Time, Throttle, EngineOn, GimbalX, GimbalY, GridX, GridY.
    """

    self.mpc.solve()

    # print (self.mpc.z.value)  