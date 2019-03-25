from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt


m = GEKKO()
n = 100
m.time = np.linspace(0, 10, n)

# Constants
g = m.Const(value=9.8)
drymass = m.Const(value=1000)

# ---- Control --------------------------------------------------
# Thrust
Impulse = np.zeros(len(m.time))  # Thrust Step test
Impulse[int(0.2*n):int(0.4*n)] = 0.5e5
Impulse[int(0.7*n):int(0.9*n)] = 1.4e5

Thrust = m.Param(value=Impulse)
mass = m.Param(value=1000)

# Vectorize Thrust
Pi = 3.14159265359
Gx, Gy = np.zeros(len(m.time)), np.zeros(len(m.time))  # Gimbal Step test
Gx[int(0.2*n):int(0.3*n)] = Pi/2.0
Gx[int(0.8*n):int(0.9*n)] = -Pi/2.0
Gy[int(0.3*n):int(0.4*n)] = Pi/4.0
Gy[int(0.7*n):int(0.8*n)] = -Pi/2.0
Pi = m.Const(value=Pi)
Gimbalx = m.Param(value=Gx)  # Angle from linear thrust in x direction
Gimbaly = m.Param(value=Gy)  # Angle from linear thrust in y direction

# Thrust with respect to coordinate fixed to rocket.
Thrustx_i = m.Intermediate(Thrust*m.sin(Gimbalx))
Thrusty_i = m.Intermediate(Thrust*m.sin(Gimbaly))
Thrustz_i = m.Intermediate(Thrust*(m.cos(Gimbalx)+m.cos(Gimbaly))/2.0)
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
I_rocket = m.Intermediate((1.0/12.0)*mass*L**2)  # Moment of inertia
d = m.Intermediate(L-I_rocket)  # Distance from moment of inertia
# ---- Angular --------------------------------------------------
tau_x = m.Intermediate(Thrustx_i*d)  # x Torque
tau_y = m.Intermediate(Thrusty_i*d)  # y Torque

θ_x = m.Var(value=0)  # x angle
θ_y = m.Var(value=0)  # y angle
w_x = m.Var(value=0)  # Rotational velocity, x direction
w_y = m.Var(value=0)
m.Equation(w_x.dt() == tau_x/I_rocket)
m.Equation(w_y.dt() == tau_y/I_rocket)
m.Equation(θ_x.dt() == w_x)
m.Equation(θ_y.dt() == w_y)

# ---- Angular --------------------------------------------------

# ---- Thrust Transformation -----------------------------------

θ_x_sign = m.Intermediate(m.abs(θ_x) / θ_x)
θ_y_sign = m.Intermediate(m.abs(θ_y) / θ_y)
θ_x_2 = m.Intermediate(-θ_x_sign*m.abs(m.abs(θ_x) - (Pi/2)))  # Complementary Angle
θ_y_2 = m.Intermediate(-θ_y_sign*m.abs(m.abs(θ_y) - (Pi/2)))  # Complementary Angle
Thrustz = m.Intermediate((Thrustz_i*m.cos(θ_x)+Thrustz_i*m.cos(θ_y))/2.0+Thrustx_i*m.cos(θ_x_2)+Thrusty_i*m.cos(θ_y_2))
Thrustx = m.Intermediate(Thrustz_i*m.sin(θ_x)+Thrustx_i*m.sin(θ_x_2))
Thrusty = m.Intermediate(Thrustz_i*m.sin(θ_y)+Thrusty_i*m.sin(θ_y_2))
# ---- Thrust Transformation -----------------------------------

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
m.Equation(vz.dt() == -g + (Thrustz_i-(m.abs(vz)/vz)*Dragz)/mass)  # Replace Thrustz_i with Thrustz (currently broke)
m.Equation(vy.dt() == 0 + (Thrusty_i-(m.abs(vy)/vy)*Dragy)/mass)
m.Equation(vx.dt() == 0 + (Thrustx_i-(m.abs(vx)/vx)*Dragx)/mass)

# ---- Main Newtonian Movement ----------------------------------

m.options.NODES = 3
m.options.IMODE = 4  # Just simulation for now, but the ultimate plan is for this to control
m.solve()

plt.subplot(6, 1, 1)
plt.plot(m.time, z.value, label='Altitude')
plt.legend(loc='best')
plt.subplot(6,1,2)
plt.plot(m.time, vz.value, label='vz')
plt.legend(loc='best')
plt.subplot(6,1,3)
plt.plot(m.time, x.value, 'r', label='x')
plt.plot(m.time, y.value, 'b', label='y')
plt.legend(loc='best')
plt.subplot(6,1,4)
plt.plot(m.time, vx.value, 'r', label='vx')
plt.plot(m.time, vy.value, 'b', label='vy')
plt.legend(loc='best')
plt.subplot(6, 1, 5)
plt.plot(m.time, Thrust.value, 'k--', label='Thrust')
plt.legend(loc='best')
plt.subplot(6, 1, 6)
plt.plot(m.time, Gimbalx.value, 'r--', label='Gimbal x')
plt.plot(m.time, Gimbaly.value, 'b--', label='Gimbal y')
plt.show()