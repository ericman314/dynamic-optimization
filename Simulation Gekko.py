import numpy as np
from gekko import GEKKO
import matplotlib.pyplot as plt


# Initialize GEKKO model
m = GEKKO()

# time
n = 100
# tf = m.FV(value=0)
m.time = np.linspace(0, 10, n)
last = np.zeros(n)
last[-1] = 1
last = m.Param(value=last)

# Parameters
final = np.zeros(np.size(m.time))
drymass = m.Param(25000)  # Rocket mass w/out fuel
propmass = m.Var(10000, lb = 0)

# Manipulated variable
Thrusty = m.MV(value=0, lb = 0, ub = 1e6)
# Thrustx = m.MV(value=0)
Thrusty.STATUS = 1
# Thrustx.STATUS = 1

# Variables
# theta = m.Var(value=0)
g = m.Param(value=9.8)

# Controlled Variable
# Position
y = m.Var(value=1000, lb=0)  # Lower bound is the ground
# x = m.Var(value=0)

# Velocity
vy = m.Var(value=-100)       # Initially falling at 100 m/s
# vx = m.Var(value=0)

# Acceleration
# ax = m.Var(value=0)
# ay = m.Var(value=-9.8)

totalmass = m.Intermediate(drymass + propmass)

# Equations -----------------------------------------------------------------------------------------------------------

#time
#m.Equation(t)

# Position
# m.Equation(x.dt() == vx)
m.Equation(y.dt() == vy)

# Velocity
# m.Equation(vx.dt() == ax)
ay = m.Intermediate(Thrusty/totalmass - g)
m.Equation(vy.dt() == ay)

# Acceleration
# m.Equation(ax == Thrustx/totalmass)

m.Equation(propmass.dt() == -Thrusty*1e-3)

# Objective
m.Obj(last*(y**2 + vy**2))
# m.Obj(last*(vx**2 + x**2))
m.Obj(-last*propmass)
# m.Obj(0.001 * (ax**2) + 0.001*(ay**2))

#%% Tuning
#global
m.options.IMODE = 5

#%% Solve
m.solve()

#%% Plot solution
plt.figure()
plt.subplot(3, 2, 1)
# plt.plot(m.time, ax.value, 'r-', linewidth=1, label='ax')
plt.plot(m.time, ay.value, 'b-', linewidth=1, label='ay')
plt.ylabel('Thrust')
plt.legend(loc='best')

plt.subplot(3, 2, 2)
# plt.plot(m.time, vx.value, 'b--', linewidth=1, label='vx')
plt.plot(m.time, vy.value, 'r--', linewidth=1, label='vy')
plt.legend(loc='best')
plt.ylabel('Velocity')

plt.subplot(3, 2, 3)
plt.plot(m.time, y.value, 'g:', linewidth=1)
plt.legend(['y'], loc='best')
plt.ylabel('Position')

plt.subplot(3, 2, 4)
# plt.plot(m.time, x.value, 'r:', linewidth=1)
plt.legend(['x'], loc='best')
plt.ylabel('Position')

plt.subplot(3, 2, 5)
plt.plot(m.time, propmass.value)

plt.subplot(3, 2, 6)
plt.plot(m.time, Thrusty.value)


plt.show()