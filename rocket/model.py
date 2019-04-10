from time import sleep
from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import random
import os.path


def getModel():

  m = GEKKO()
  m.options.NODES = 3
  # Do not set IMODE here, as the same model might be used for MPC and MHE

  # Constants
  g = m.Const(value=9.8)
  drymass = m.Const(value=27200)

  m.Throttle = m.MV(value=1.0, lb=0.57, ub=1.0)
  m.EngineOn = m.MV(value=0, lb=0, ub=1, integer=True)
  m.propMass = m.Var(value=1000)

  m.f9ThrustSL = m.Const(7607000 / 9)     # N, per engine
  m.f9ThrustVac = m.Const(8227000 / 9)    # N, per engine
    

  Pi = m.Const(value=np.pi)
  m.Gimbalx = m.MV(value=0)  # Angle from linear Thrust in x direction
  m.Gimbaly = m.MV(value=0)  # Angle from linear thrust in y direction

  m.Gridx = m.MV(value=0)
  m.Gridy = m.MV(value=0)

  # Position
  m.x = m.CV(value=0)
  m.y = m.CV(value=0)
  m.z = m.CV(value=0)

  # Velocity
  m.vx = m.CV(value=0)
  m.vy = m.CV(value=0)
  m.vz = m.CV(value=0)

  # Angular position and velocity
  m.θ_x = m.CV(value=0)  # x angle
  m.θ_y = m.CV(value=0)  # y angle
  m.w_x = m.CV(value=0)  # Rotational velocity, x direction (Initial conditions for angular velocity not supported yet)
  m.w_y = m.CV(value=0)
 
  # Adjustable parameters
  pointingAuthority = m.FV(4000)
  liftAuthority = m.FV(300)
  dragAuthority = m.FV(1.5)
  gimbalAuthority = m.FV(0.28)
  gridAuthority = m.FV(12)
  Ifactorempirical = m.FV(value=251.0)

  vRelAir2 = m.Intermediate(m.vx**2 + m.vy**2 + m.vz**2)
  vRelAirMag = m.Intermediate( m.sqrt(vRelAir2) )
  vRelAirNormx = m.Intermediate(m.vx / vRelAirMag)
  vRelAirNormy = m.Intermediate(m.vy / vRelAirMag)
  vRelAirNormz = m.Intermediate(m.vz / vRelAirMag)

  # Atmospheric density and pressure
  ρ = m.Intermediate( 1.2205611857638659 * m.exp(-0.00009107790874911096 * m.z + -1.8783521651107734e-9 * m.z**2 ))  # Density of Air
  press = m.Intermediate( 101325 * m.exp(-0.00011890154532889426 * m.z + -1.4298587512183478e-9 * m.z**2 )) # Pressure of air


  dynPress = m.Intermediate(0.5 * ρ * vRelAir2)

  I_rocket = m.Intermediate( Ifactorempirical*(m.propMass+drymass) )  # Moment of inertia
  

  # Force the rocket point in the direction of travel
  pointingErrorX = m.Intermediate(m.vx / m.sqrt(vRelAir2) + m.θ_x)
  pointingErrorY = m.Intermediate(m.vy / m.sqrt(vRelAir2) + m.θ_y)
  pointingTauX = m.Intermediate(-pointingErrorX * pointingAuthority * dynPress)
  pointingTauY = m.Intermediate(-pointingErrorY * pointingAuthority * dynPress)

  # Rocket body should generate some lift if not pointing exactly correct
  Liftx = m.Intermediate(-pointingErrorX * liftAuthority * dynPress)
  Lifty = m.Intermediate(-pointingErrorY * liftAuthority * dynPress)

  m.AOA = m.Intermediate( m.sqrt(pointingErrorX**2 + pointingErrorY**2) )

  # Slow the rocket down in the direction of travel (drag)
  dragArea = m.Intermediate(10.8 + 163.5 * m.AOA)
  dragForce = m.Intermediate(dynPress * dragArea * dragAuthority)

  Dragx = m.Intermediate( -dragForce * vRelAirNormx)
  Dragy = m.Intermediate( -dragForce * vRelAirNormy)
  Dragz = m.Intermediate( -dragForce * vRelAirNormz)

  # Thrust in world coords
  m.Thrust = m.Intermediate(m.Throttle * m.EngineOn * (m.f9ThrustSL * press / 101325 + m.f9ThrustVac * (1 - press / 101325)))
  Thrustvecx = m.Intermediate(m.Gimbalx * np.pi / 180 + m.θ_x)
  Thrustvecy = m.Intermediate(m.Gimbaly * np.pi / 180 + m.θ_y)
  Thrustx = m.Intermediate(1 * m.Thrust * Thrustvecx)
  Thrusty = m.Intermediate(1 * m.Thrust * Thrustvecy)
  Thrustz = m.Intermediate(1 * m.Thrust * m.sqrt(1 - Thrustvecx**2 - Thrustvecy**2))

  # Engine gimballing exerts a torque on the rocket
  gimbalTauX = m.Intermediate(-m.Thrust * m.Gimbalx * gimbalAuthority)
  gimbalTauY = m.Intermediate(-m.Thrust * m.Gimbaly * gimbalAuthority)

  # Grid fins exert a torque on the rocket (technically, it alters the pointing error, but this should average out to be correct)
  gridTauX = m.Intermediate(-m.Gridx * dynPress * gridAuthority)
  gridTauY = m.Intermediate(-m.Gridy * dynPress * gridAuthority)

  # Equation - Newtonian
  m.Equation(m.z.dt() == m.vz)
  m.Equation(m.y.dt() == m.vy)
  m.Equation(m.x.dt() == m.vx)

  m.Equation(m.vx.dt() ==  0 + (Dragx + Thrustx + Liftx) / (m.propMass+drymass))
  m.Equation(m.vy.dt() ==  0 + (Dragy + Thrusty + Lifty) / (m.propMass+drymass))
  m.Equation(m.vz.dt() == -g + (Dragz + Thrustz) / (m.propMass+drymass))

  m.Equation(m.w_x.dt()*I_rocket == pointingTauX + gimbalTauX + gridTauX)
  m.Equation(m.w_y.dt()*I_rocket == pointingTauY + gimbalTauY + gridTauY)
  m.Equation(m.θ_x.dt() == m.w_x)
  m.Equation(m.θ_y.dt() == m.w_y)

  m.Equation(m.propMass.dt() == -300 * m.Throttle * m.EngineOn) 

  m.pointingTauX = pointingTauX
  m.Dragz = Dragz
  m.Thrustvecx = Thrustvecx
  m.Thrustvecy = Thrustvecy

  return m

