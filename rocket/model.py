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

  m.Throttle = m.Param(value=0)
  m.propMass = m.Var(value=1000)

  m.f9ThrustSL = m.Const(7607000 / 9)     # N, per engine
  m.f9ThrustVac = m.Const(8227000 / 9)    # N, per engine
    

  Pi = m.Const(value=np.pi)
  m.Gimbalx = m.Param(value=0)  # Angle from linear Thrust in x direction
  m.Gimbaly = m.Param(value=0)  # Angle from linear thrust in y direction

  # Position
  m.x = m.Var(value=0)
  m.y = m.Var(value=0)
  m.z = m.Var(value=0)

  # Velocity
  m.vx = m.Var(value=0)
  m.vy = m.Var(value=0)
  m.vz = m.Var(value=0)

  # Atmospheric density and pressure
  ρ = m.Intermediate( 1.2205611857638659 * m.exp(-0.00009107790874911096 * m.z + -1.8783521651107734e-9 * m.z**2 ))  # Density of Air
  press = m.Intermediate( 101325 * m.exp(-0.00011890154532889426 * m.z + -1.4298587512183478e-9 * m.z**2 )) # Pressure of air

  # Prop consumption (each engine consumes 300kg/s of propellent at 100% throttle)
  m.Equation(m.propMass.dt() == -300 * m.Throttle) 
  m.Thrust = m.Intermediate(m.Throttle * (m.f9ThrustSL * press / 101325 + m.f9ThrustVac * (1 - press / 101325)))

  # Thrust with respect to coordinate fixed to rocket.
  Thrustx_i = m.Intermediate(m.Thrust*m.sin(-m.Gimbalx*np.pi/180))
  Thrusty_i = m.Intermediate(m.Thrust*m.sin(m.Gimbaly*np.pi/180))
  Thrustz_i = m.Intermediate(m.Thrust*(1 - m.sin(-m.Gimbalx*np.pi/180)**2 - m.sin(m.Gimbaly*np.pi/180)**2))

  # ---- Control --------------------------------------------------

  # ---- Drag -----------------------------------------------------
  # These are cross sections, they should be Var in the future
  Ax = m.Const(value=4)
  Ay = m.Const(value=4)
  Az = m.Const(value=0.2)
  Cd = m.Const(value=0.5)  # Similar for sphere and cone shape

  # ---- Drag -----------------------------------------------------
  L = m.Const(value=8.0)  # Length of Rocket
  Ifactorempirical = m.Param(value=251.0)
  # I_rocket = m.Intermediate((1.0/12.0)*(m.propMass+drymass)*L**2)  # Moment of inertia
  I_rocket = m.Intermediate( Ifactorempirical*(m.propMass+drymass) )  # Moment of inertia
  # ---- Angular --------------------------------------------------
  tau_x = m.Intermediate(Thrustx_i * 15.5)  # x Torque  (15.5 = distance between COM and engines)
  tau_y = m.Intermediate(Thrusty_i * 15.5)  # y Torque

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

  # With theta_x, we are exchanging x and z.
  # With theta_y, we are exchanging y and z

  # Rotation matrix for rotations on the y-axis (theta_x):
  #
  # | cos(x)  0   -sin(x) |
  # |   0     1      0    |
  # | sin(x)  0    cos(x) |
  #
  # Rotation matrix for rotations on the x-axis (theta_y):
  #
  # |   1    0       0    |
  # |   0  cos(y) -sin(y) |
  # |   0  sin(y)  cos(y) |

  # We'll rotate x, then y. So we need to compute mat_y * mat_x
  # |   1    0       0    |
  # |   0  cos(y) -sin(y) |
  # |   0  sin(y)  cos(y) |

  # |   1    0       0    |   | cos(x)  0   -sin(x) |   |  cos(x)         0      -sin(x)        |
  # |   0  cos(y) -sin(y) | * |   0     1      0    | = | -sin(x)*sin(y)  cos(y) -cos(x)*sin(y) |
  # |   0  sin(y)  cos(y) |   | sin(x)  0    cos(x) |   |  sin(x)*cos(y)  sin(y)  cos(x)*cos(y) |
  

  # TODO: Is it faster to make cos(theta_x), etc., intermediates?

  # Oops I got theta_x and theta_y backwards. Or maybe I multiplied them in the wrong order.

  Thrustx = m.Intermediate(Thrustx_i * m.cos(-m.θ_x) - Thrustz_i * m.sin(-m.θ_x))
  Thrusty = m.Intermediate(Thrustx_i * -m.sin(-m.θ_x) * m.sin(-m.θ_y) + Thrusty_i * m.cos(-m.θ_y) - Thrustz_i * -m.cos(-m.θ_x) * m.sin(-m.θ_y))
  Thrustz = m.Intermediate(Thrustx_i * m.sin(-m.θ_x) * m.cos(-m.θ_y) + Thrusty_i * m.sin(-m.θ_y) + Thrustz_i * m.cos(-m.θ_x) * m.cos(-m.θ_y))

  # ---- Main Newtonian Movement ----------------------------------
  

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

