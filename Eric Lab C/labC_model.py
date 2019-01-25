from gekko import GEKKO
import numpy as np


def getModel(times, heater1Power, heater2Power, Tmeas1, Tmeas2):

  # Initialize model for dynamic simulation
  m = GEKKO()
  m.options.IMODE = 5

  # Time grid (seconds)
  m.time = times

  # Manipulated variables
  Q1    = m.MV(value = heater1Power)   # heater power (0-100%)
  Q2    = m.MV(value = heater2Power)

  # Constant parameters
  mass  = m.Param(0.004)   # mass (kg)
  Cp    = m.Param(500)   # heat capacity (J/kg K)
  A     = m.Param(0.0010)  # surface area not between sinks (m^2)
  #As    = m.Param(0.0002)  # surface area between sinks (m^2)
  eps   = m.Param(0.9)   # heater emissivity
  sigma = m.Const(5.67e-8)   # Stefan-Boltzman constant
  
  # Adjustable parameters
  Ta    = m.FV(value = 300, lb = 280, ub = 310)   # ambient temperature (K)
  U     = m.FV(value = 4, lb = -15.0, ub = 15)   # heat transfer coefficient (W/m^2 K)
  As    = m.FV(value = 0.0002, lb = 1e-6, ub = 0.01)
  alpha1 = m.FV(value = 0.01, lb = 0.0, ub = 0.1)   # heater 1 constant (W / % power) 
  alpha2 = m.FV(value = 0.0075, lb = 0.0, ub = 0.1) # heater 2 constant (W / % power)

  # Allow the solver to adjust these
  Ta.STATUS = 1
  U.STATUS = 1
  alpha1.STATUS = 1
  alpha2.STATUS = 1
  As.STATUS = 1

  # Measured variables
  T1 = m.CV(value = Tmeas1)
  T2 = m.CV(value = Tmeas2)

  # Set these as the objective
  T1.FSTATUS = 1
  T2.FSTATUS = 1


  # Add some unmeasured state variables representing the temperature on the heat sink at various points
  # This will simulate heat conduction from the heater to the thermistor
  # Initial value is the measured temperature
  Tint1 = m.SV(value = Tmeas1)
  Tint2 = m.SV(value = Tmeas2)

  # Conduction terms (loose definition, it just adds a time-delay to the thermistor's response)
  cond1 = m.FV(0.01, lb = 0.01, ub = 10)
  cond2 = m.FV(0.01, lb = 0.01, ub = 10)
  cond1.STATUS = 1
  cond2.STATUS = 1


  # Contributions from each heat source
  Q1conv = m.Intermediate( U * A * (Ta - T1) )
  Q1rad  = m.Intermediate( eps * sigma * A * (Ta**4 - T1**4) )
  Q1heat = m.Intermediate( Q1 * alpha1 )
  Q2conv = m.Intermediate( U * A * (Ta - T2) )
  Q2rad  = m.Intermediate( eps * sigma * A * (Ta**4 - T2**4) )
  Q2heat = m.Intermediate( Q2 * alpha2 )
  Q12conv= m.Intermediate( U * As * (T2 - T1) )
  Q12rad = m.Intermediate( eps * sigma * As * (T2**4 - T1**4) )
  Q1tot  = m.Intermediate( Q1conv + Q1rad + Q1heat + Q12conv + Q12rad )
  Q2tot  = m.Intermediate( Q2conv + Q2rad + Q2heat - Q12conv - Q12rad )
  
  # Transient energy balance
  # m.Equation(T1.dt() == Q1tot / (mass * Cp))
  # m.Equation(T2.dt() == Q2tot / (mass * Cp))
  m.Equation(Tint1.dt() == Q1tot / (mass * Cp))
  m.Equation(Tint2.dt() == Q2tot / (mass * Cp))
  
  # Very simple one-way conduction from heater to thermistor, in order to add a time-delay to the model
  m.Equation(T1.dt() == cond1 * (Tint1 - T1))
  m.Equation(T2.dt() == cond2 * (Tint2 - T2))

  # Minimize (meas-pred)^2
  m.options.EV_TYPE = 2

  m.T1, m.T2 = T1, T2
  m.Q1conv, m.Q2conv = Q1conv, Q2conv
  m.Q1rad, m.Q2rad = Q1rad, Q2rad
  m.Q1heat, m.Q2heat = Q1heat, Q2heat
  m.Q1tot, m.Q2tot = Q1tot, Q2tot
  m.Q12conv, m.Q12rad = Q12conv, Q12rad
  m.alpha1, m.alpha2 = alpha1, alpha2
  m.Ta = Ta
  m.U = U
  m.As = As
  m.cond1, m.cond2 = cond1, cond2
  m.Tint1 = Tint1
  m.Tint2 = Tint2

  return m

