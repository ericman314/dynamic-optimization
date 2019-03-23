from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt


m = GEKKO()
n = 1000
m.time = np.linspace(0, 25, n)

# Constants
g = m.Const(value=9.8)
drymass = m.Const(value=1000)

# ---- Control --------------------------------------------------
# Thrust
Impulse = np.zeros(len(m.time))
Impulse[int(0.2*n):int(0.4*n)] = 1e4
Impulse[int(0.7*n):int(0.9*n)] = 2.3e5

Thrust = m.Param(value=Impulse)
mass = m.Param(value=1000)

# Vectorize Thrust
Gimbalx = m.Param(value=1.57)  # Angle from linear thrust in x direction
Gimbaly = m.Param(value=0.3)  # Angle from linear thrust in y direction

Thrustx = m.Intermediate(Thrust*m.sin(Gimbalx))
Thrusty = m.Intermediate(Thrust*m.sin(Gimbaly))
Thrustz = m.Intermediate(Thrust*(m.cos(Gimbalx)+m.cos(Gimbaly))/2.0)
# ---- Control --------------------------------------------------

# ---- Drag -----------------------------------------------------
ρ = m.Const(value=1.225)  # Density of Air
# These are cross sections, they should be Var in the future
Ax = m.Const(value=4)
Ay = m.Const(value=4)
Az = m.Const(value=0.2)
Cd = m.Const(value=0.5)  # Similar for sphere and cone shape

# ---- Drag -----------------------------------------------------

# ---- Angular --------------------------------------------------
# I had this all coded, but it got deleted because I don't know github so Im going to do this tomorrow or sunday.
# ---- Angular --------------------------------------------------

# ---- Main Newtonian Movement ----------------------------------
# Position
x = m.Var(value=5)
y = m.Var(value=10)
z = m.Var(value=3000)

# Velocity
vx = m.Var(value=1)
vy = m.Var(value=-3)
vz = m.Var(value=-100)

# Equation - Newtonian
m.Equation(z.dt() == vz)
m.Equation(y.dt() == vy)
m.Equation(x.dt() == vx)

# Drag
Dragx = m.Intermediate(Cd*ρ*(vx**2)*Ax/2.0)
Dragy = m.Intermediate(Cd*ρ*(vy**2)*Ay/2.0)
Dragz = m.Intermediate(Cd*ρ*(vz**2)*Az/2.0)

# Acceleration
# abs/v = the direction of the velocity
m.Equation(vz.dt() == -g + (Thrustz-(m.abs(vz)/vz)*Dragz)/mass)
m.Equation(vy.dt() == 0 + (Thrusty-(m.abs(vy)/vy)*Dragy)/mass)
m.Equation(vx.dt() == 0 + (Thrustx-(m.abs(vx)/vx)*Dragx)/mass)

# ---- Main Newtonian Movement ----------------------------------

m.options.IMODE = 4
m.solve()

plt.subplot(4, 1, 1)
plt.plot(m.time, z.value, label='Altitude')
plt.legend(loc='best')
plt.subplot(4,1,2)
plt.plot(m.time, vz.value, label='vz')
plt.legend(loc='best')
plt.subplot(4,1,3)
plt.plot(m.time, x.value, 'r', label='x')
plt.plot(m.time, y.value, 'b', label='y')
plt.legend(loc='best')
plt.subplot(4,1,4)
plt.plot(m.time, vx.value, 'r', label='vx')
plt.plot(m.time, vy.value, 'b', label='vy')
plt.legend(loc='best')
plt.show()