from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt
import random

m = GEKKO()
n = 100
m.time = np.linspace(0, 10, n)

# Constants
g = m.Const(value=9.8)
drymass = m.Const(value=1000)

# ---- Control --------------------------------------------------
# Thrust
Impulse = np.zeros(len(m.time))  # Thrust Step test
Impulse[int(0.2*n):int(0.4*n)] = 0 #0.5e5
Impulse[int(0.7*n):int(0.9*n)] = 0 #1.4e5

Thrust = m.Param(value=Impulse)
mass = m.Param(value=1000)

# Vectorize Thrust
Pi = 3.14159265359
Gx, Gy = np.zeros(len(m.time)), np.zeros(len(m.time))  # Gimbal Step test
Gx[int(0.2*n):int(0.3*n)] = 0
Gx[int(0.8*n):int(0.9*n)] = 0
Gy[int(0.3*n):int(0.33*n)] = 0 #0.01
Gy[int(0.7*n):int(0.72*n)] = 0 # -0.01
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
m.Equation(w_x.dt()*I_rocket == tau_x)
m.Equation(w_y.dt()*I_rocket == tau_y)
m.Equation(θ_x.dt() == w_x)
m.Equation(θ_y.dt() == w_y)

# ---- Angular --------------------------------------------------

# ---- Thrust Transformation -----------------------------------

θ_x_2 = m.Intermediate(m.abs(m.abs(θ_x) - (Pi/2)))  # Complementary Angle
θ_y_2 = m.Intermediate(m.abs(m.abs(θ_y) - (Pi/2)))  # Complementary Angle
Thrustz = m.Intermediate((Thrustz_i*m.cos(θ_x)+Thrustz_i*m.cos(θ_y))/2.0-Thrustx_i*m.sin(θ_x)-Thrusty_i*m.sin(θ_y))
Thrustx = m.Intermediate(Thrustz_i*m.sin(θ_x)+Thrustx_i*m.sin(θ_x))
Thrusty = m.Intermediate(Thrustz_i*m.sin(θ_y)+Thrusty_i*m.sin(θ_y))
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
m.Equation(vz.dt() == -g + (Thrustz-(m.abs(vz)/vz)*Dragz)/mass)  # Replace Thrustz_i with Thrustz (currently broke)
m.Equation(vy.dt() == 0 + (Thrusty-(m.abs(vy)/vy)*Dragy)/mass)
m.Equation(vx.dt() == 0 + (Thrustx-(m.abs(vx)/vx)*Dragx)/mass)

# ---- Main Newtonian Movement ----------------------------------

m.options.NODES = 3
m.options.IMODE = 4  # Just simulation for now, but the ultimate plan is for this to control
m.solve(Remote=False)



plt.figure(num=2, figsize=(10,8))
plt.subplot2grid((15,2),(0,0), rowspan=3)
plt.plot(m.time, z.value)
plt.ylabel('Altitude '+r'$(m)$')

plt.subplot2grid((15,2),(0,1), rowspan=5)
plt.plot(m.time, w_x.value, label=r'$\omega_x$')
plt.plot(m.time, w_y.value, label=r'$\omega_y$')
plt.legend(loc='best')
plt.ylabel('Rotational velocity '+ r'$(\frac{rotations}{sec})$')

plt.subplot2grid((15,2),(3,0), rowspan=3)
plt.plot(m.time, vz.value)
plt.ylabel('Fall velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(5,1), rowspan=5)
plt.plot(m.time, θ_x.value, 'r', label=r'$θ_x$')
plt.plot(m.time, θ_y.value, 'b', label=r'$θ_y$')
plt.legend(loc='best')
plt.ylabel('Angle '+r'$(rad)$')

plt.subplot2grid((15,2),(6,0), rowspan=3)
plt.plot(m.time, vx.value, 'r', label=r'$v_x$')
plt.plot(m.time, vy.value, 'b', label=r'$v_y$')
plt.legend(loc='best')
plt.ylabel('Velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(9,0), rowspan=3)
plt.plot(m.time, x.value, 'r', label='x')
plt.plot(m.time, y.value, 'b', label='y')
plt.ylabel('Position '+r'$(m)$')
plt.legend(loc='best')

plt.subplot2grid((15,2),(10,1), rowspan=5)
plt.plot(m.time, Gimbalx.value, 'r--', label=r'$Gimbal_X$')
plt.plot(m.time, Gimbaly.value, 'b--', label=r'$Gimbal_Y$')
plt.legend(loc='best')
plt.ylabel('Gimbal '+r'$(rad)$')
plt.xlabel('Time')

plt.subplot2grid((15,2),(12,0), rowspan=3)
plt.plot(m.time, Thrust.value, 'k--')
plt.ylabel('Thrust')
plt.xlabel('Time')

plt.tight_layout()
plt.subplots_adjust(top=0.95,wspace=0.3)
plt.show()