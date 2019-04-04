from time import sleep
from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import random
import os.path

# Old hardcoded steptests from Simulation Gekko.py
# # ---- Control --------------------------------------------------
# # Thrust
# Impulse = np.zeros(len(m.time))  # Thrust Step test
# Impulse[int(0.2*n):int(0.4*n)] = 0 #0.5e5
# Impulse[int(0.7*n):int(0.9*n)] = 0 #1.4e5


# # Vectorize Thrust
# Pi = 3.14159265359
# Gx, Gy = np.zeros(len(m.time)), np.zeros(len(m.time))  # Gimbal Step test
# Gx[int(0.2*n):int(0.3*n)] = 0
# Gx[int(0.8*n):int(0.9*n)] = 0
# Gy[int(0.3*n):int(0.33*n)] = 0 #0.01
# Gy[int(0.7*n):int(0.72*n)] = 0 # -0.01


def getModel():

  m = GEKKO()
  m.options.NODES = 3
  # Do not set IMODE here, as the same model might be used for MPC and MHE

  # Constants
  g = m.Const(value=9.8)
  drymass = m.Const(value=27200)

  m.Thrust = m.Param(value=0)
  m.propMass = m.Var(value=1000)

  Pi = m.Const(value=np.pi)
  m.Gimbalx = m.Param(value=0)  # Angle from linear Thrust in x direction
  m.Gimbaly = m.Param(value=0)  # Angle from linear thrust in y direction

  # Prop consumption (each engine consumes 300kg/s of propellent at 100% throttle)
  m.Equation(m.propMass.dt() == -300 * m.Thrust) 

  # Thrust with respect to coordinate fixed to rocket.
  Thrustx_i = m.Intermediate(m.Thrust*m.sin(m.Gimbalx))
  Thrusty_i = m.Intermediate(m.Thrust*m.sin(m.Gimbaly))
  Thrustz_i = m.Intermediate(m.Thrust*(m.cos(m.Gimbalx)+m.cos(m.Gimbaly))/2.0)
  # ---- Control --------------------------------------------------

  # ---- Drag -----------------------------------------------------
  ρ = m.Const(value=1.225)  # Density of Air
  # These are cross sections, they should be Var in the future
  Ax = m.Const(value=4)
  Ay = m.Const(value=4)
  Az = m.Const(value=0.2)
  Cd = m.Const(value=0.5)  # Similar for sphere and cone shape

  # ---- Drag -----------------------------------------------------
  L = m.Const(value=8.0)  # Length of Rocket
  I_rocket = m.Intermediate((1.0/12.0)*(m.propMass+drymass)*L**2)  # Moment of inertia
  d = m.Intermediate(L-I_rocket)  # Distance from moment of inertia
  # ---- Angular --------------------------------------------------
  tau_x = m.Intermediate(Thrustx_i*d)  # x Torque
  tau_y = m.Intermediate(Thrusty_i*d)  # y Torque

  m.θ_x = m.Var(value=0)  # x angle
  m.θ_y = m.Var(value=0)  # y angle
  m.w_x = m.Var(value=0)  # Rotational velocity, x direction (Initial conditions for angular velocity not supported yet)
  m.w_y = m.Var(value=0)
  m.Equation(m.w_x.dt()*I_rocket == tau_x)
  m.Equation(m.w_y.dt()*I_rocket == tau_y)
  m.Equation(m.θ_x.dt() == m.w_x)
  m.Equation(m.θ_y.dt() == m.w_y)


  # ---- Thrust Transformation -----------------------------------
  θ_x_2 = m.Intermediate(m.abs(m.abs(m.θ_x) - (Pi/2)))  # Complementary Angle
  θ_y_2 = m.Intermediate(m.abs(m.abs(m.θ_y) - (Pi/2)))  # Complementary Angle
  Thrustz = m.Intermediate((Thrustz_i*m.cos(m.θ_x)+Thrustz_i*m.cos(m.θ_y))/2.0-Thrustx_i*m.sin(m.θ_x)-Thrusty_i*m.sin(m.θ_y))
  Thrustx = m.Intermediate(Thrustz_i*m.sin(m.θ_x)+Thrustx_i*m.sin(m.θ_x))
  Thrusty = m.Intermediate(Thrustz_i*m.sin(m.θ_y)+Thrusty_i*m.sin(m.θ_y))

  # ---- Main Newtonian Movement ----------------------------------
  # Position
  m.x = m.Var(value=0)
  m.y = m.Var(value=0)
  m.z = m.Var(value=0)

  # Velocity
  m.vx = m.Var(value=0)
  m.vy = m.Var(value=0)
  m.vz = m.Var(value=0)

  # Equation - Newtonian
  m.Equation(m.z.dt() == m.vz)
  m.Equation(m.y.dt() == m.vy)
  m.Equation(m.x.dt() == m.vx)

  # Drag
  Dragx = m.Intermediate(Cd*ρ*(m.vx**2)*Ax/2.0)
  Dragy = m.Intermediate(Cd*ρ*(m.vy**2)*Ay/2.0)
  Dragz = m.Intermediate(Cd*ρ*(m.vz**2)*Az/2.0)

  # Acceleration
  # abs/v = the direction of the velocity
  m.Equation(m.vz.dt() == -g + (Thrustz-(m.abs(m.vz)/m.vz)*Dragz)/(m.propMass+drymass))  # Replace Thrustz_i with Thrustz (currently broke)
  m.Equation(m.vy.dt() == 0 + (Thrusty-(m.abs(m.vy)/m.vy)*Dragy)/(m.propMass+drymass))
  m.Equation(m.vx.dt() == 0 + (Thrustx-(m.abs(m.vx)/m.vx)*Dragx)/(m.propMass+drymass))

  return m

