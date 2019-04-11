import numpy as np
import matplotlib.pyplot as plt
import os

from model import getModel

# Load the results from the simulation
simFilename = '20km-10%prop-500mpsdown_gridRotate'

# Time (sec), X (m), Y (m), Z (m), Roll (deg), Yaw (deg), Pitch (deg), Xdot (m/s), Ydot (m/s), Zdot (m/s), Prop (kg), Throttle (0-1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg), GeeAxial (g), GeeLateral (g), AOA (deg)
sim = np.loadtxt(os.path.join('simulationData', simFilename + '.csv'), delimiter=',')
simTime = sim[:,0]
simX = sim[:,1]
simY = sim[:,2]
simZ = sim[:,3]
simRoll = sim[:,4]
simYaw = sim[:,5]
simPitch = sim[:,6]
simXdot = sim[:,7]
simYdot = sim[:,8]
simZdot = sim[:,9]
simProp = sim[:,10]
simThrottle = sim[:,11]
simGimbalX = sim[:,12]
simGimbalY = sim[:,13]
simGridX = sim[:,14]
simGridY = sim[:,15]
simAOA = sim[:,18]
simEngineOn = np.array([ 1 if x > 0 else 0 for x in simThrottle ])


# Create the model
m = getModel()

m.options.COLDSTART = 2

# Set initial conditions
m.x.value = simX
m.y.value = simY
m.z.value = simZ
m.vx.value = simXdot
m.vy.value = simYdot
m.vz.value = simZdot
m.θ_x.value = simYaw * np.pi / 180  # x angle
m.θ_y.value = simPitch * np.pi / 180 # y angle
m.propMass.value = simProp[0]  
  
m.options.IMODE = 4  # Just simulation for now, but the ultimate plan is for this to control
m.time = simTime

# Set MVs
m.Throttle.value = simThrottle
m.EngineOn.value = simEngineOn
m.Gimbalx.value = simGimbalX
m.Gimbaly.value = simGimbalY
m.Gridx.value = simGridX
m.Gridy.value = simGridY

m.solve()


# plt.figure(num=2, figsize=(10,8))
plt.figure(figsize=(10,8))
plt.subplot2grid((20,2),(0,0), rowspan=4)
plt.plot(m.time, m.z.value, ':', color='royalblue', label='Model')
plt.plot(m.time, simZ, '-', color='royalblue', label='Sim')
plt.legend()
plt.ylabel('Altitude '+r'$(m)$')

plt.subplot2grid((20,2),(0,1), rowspan=5)
plt.plot(m.time, np.array(m.w_x.value)*180/np.pi, label=r'$\omega_x$')
plt.plot(m.time, np.array(m.w_y.value)*180/np.pi, label=r'$\omega_y$')
plt.legend(loc='best')
plt.ylabel('Rotational velocity '+ r'$(\frac{deg}{sec})$')

plt.subplot2grid((20,2),(4,0), rowspan=4)
plt.plot(m.time, m.vz.value, ':', color='darkorange', label='Model')
plt.plot(m.time, simZdot, '-', color='darkorange', label='Sim')
plt.legend()
plt.ylabel('Fall velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((20,2),(5,1), rowspan=5)
plt.plot(m.time, np.array(m.θ_x.value)*180/np.pi, 'r:', label=r'$θ_x$ Model')
plt.plot(m.time, np.array(m.θ_y.value)*180/np.pi, 'b:', label=r'$θ_y$ Model')
plt.plot(m.time, simYaw, 'r-', label=r'$θ_x$ Sim')
plt.plot(m.time, simPitch, 'b-', label=r'$θ_y$ Sim')
plt.legend(loc='best')
plt.ylabel('Angle '+r'$(deg)$')

plt.subplot2grid((20,2),(8,0), rowspan=4)
plt.plot(m.time, m.vx.value, 'r:', label=r'$v_x$ Model')
plt.plot(m.time, m.vy.value, 'b:', label=r'$v_y$ Model')
plt.plot(m.time, simXdot, 'r-', label=r'$v_x$ Sim')
plt.plot(m.time, simYdot, 'b-', label=r'$v_y$ Sim')
plt.legend(loc='best')
plt.ylabel('Velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((20,2),(12,0), rowspan=4)
plt.plot(m.time, m.x.value, 'r:', label='x Model')
plt.plot(m.time, m.y.value, 'b:', label='y Model')
plt.plot(m.time, simX, 'r-', label='x Sim')
plt.plot(m.time, simY, 'b-', label='y Sim')
plt.ylabel('Position '+r'$(m)$')
plt.legend(loc='best')

plt.subplot2grid((20,2),(10,1), rowspan=5)
plt.plot(m.time, m.Gimbalx.value, 'r--', label=r'$Gimbal_X$')
plt.plot(m.time, m.Gimbaly.value, 'b--', label=r'$Gimbal_Y$')
plt.plot(m.time, simGimbalX, 'r', label=r'$Gimbal_X$ model')
plt.plot(m.time, simGimbalY, 'b', label=r'$Gimbal_Y$ model')
plt.legend(loc='best')
plt.ylabel('Gimbal '+r'$(deg)$')
plt.xlabel('Time')

plt.subplot2grid((20,2),(16,0), rowspan=4)
plt.plot(m.time, m.Thrust.value, 'k--')
plt.ylabel('Thrust')
plt.xlabel('Time')

plt.subplot2grid((20,2),(15,1), rowspan=5)
plt.plot(m.time, np.asarray(m.EngineOn.value)*np.asarray(m.Throttle.value), 'k--', label='Engine (Model)')
plt.plot(m.time, simEngineOn*simThrottle, 'k', label='Engine (Simulation)')
plt.legend()
plt.ylabel('Engine')
plt.xlabel('Time')

# plt.tight_layout()
plt.subplots_adjust(top=0.95, bottom=0.07,wspace=0.3, hspace=1.8, left=0.1, right=0.98)

plt.savefig(os.path.join('plots', simFilename + ' step test.png'))

# plt.clf()
plt.figure()
plt.subplot(211)
plt.plot(m.time, np.array(m.AOA.value)*180/np.pi, 'r:', label='AOA Model')
plt.plot(m.time, simAOA, 'r-', label='AOA Sim')
plt.legend()
plt.ylabel('Angle of attack'+r'$(deg)$')

plt.subplot(212)
plt.plot(m.time, m.Gridx.value, 'b:', label='GridX Model')
plt.plot(m.time, m.Gridy.value, 'r:', label='GridY Model')
plt.plot(m.time, simGridX, 'b', label='GridX simulation')
plt.plot(m.time, simGridY, 'r', label='GridY simulation')
plt.legend(loc='best')
plt.ylabel('Grid fin position'+r'$(m)$')
plt.xlabel('Time')

plt.show()