import numpy as np
import matplotlib.pyplot as plt
import os

from model import getModel

# # Filename to read initial conditions from (don't include the .csv)
# #initFilename = '50m-drop'
# initFilename = '40km-5%prop-750mpsdown'

# # Filename to read step tests from (don't include the .csv)
# stepFilename = 'gimbalX'

# Load the results from the simulation
simFilename = '40km-5%prop-750mpsdown_none'

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


# Create the model
m = getModel()

# Set initial conditions
m.x.value = simX[0]
m.y.value = simY[0]
m.z.value = simZ[0]
m.vx.value = simXdot[0]
m.vy.value = simYdot[0]
m.vz.value = simZdot[0]
m.θ_x.value = simYaw[0]  # x angle
m.θ_y.value = simPitch[0]  # y angle
  

m.options.IMODE = 4  # Just simulation for now, but the ultimate plan is for this to control
m.time = simTime

# # Time (sec), Throttle (0-1), EngineOn (0 or 1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg)
# stepTests = np.loadtxt(os.path.join('stepTests', stepFilename + '.csv'), delimiter=',')

# # The step test file has one row for every step change, but we need one row for every timestep.
# # Create arrays to hold the MV values for each timestep.
# stepThrottle = np.zeros(m.time.size)
# stepEngineOn = np.zeros(m.time.size)
# stepGimbalX = np.zeros(m.time.size)
# stepGimbalY = np.zeros(m.time.size)
# stepGridX = np.zeros(m.time.size)
# stepGridY = np.zeros(m.time.size)

# # Assign the step test values from the file to our per-timestep arrays
# i = -1
# j = -1
# for t in m.time:
#   i += 1
#   # Find the right time

#   while j+1 < stepTests.shape[0] and stepTests[j+1][0] < t:
#     j += 1
#   stepThrottle[i] = stepTests[j, 1]
#   stepEngineOn[i] = stepTests[j, 2]
#   stepGimbalX[i] = stepTests[j, 3]
#   stepGimbalY[i] = stepTests[j, 4]
#   stepGridX[i] = stepTests[j, 5]
#   stepGridY[i] = stepTests[j, 6]

# Set MVs
m.Thrust.value = simThrottle
#m.EngineOn.value = stepEngineOn
m.Gimbalx.value = simGimbalX
m.Gimbaly.value = simGimbalY
#m.Gridx.value = stepGridX
#m.Gridy.value = stepGridY

m.solve(Remote=False)

plt.figure(num=2, figsize=(10,8))
plt.subplot2grid((15,2),(0,0), rowspan=3)
plt.plot(m.time, m.z.value, ':', color='royalblue', label='Model')
plt.plot(m.time, simZ, '-', color='royalblue', label='Sim')
plt.legend()
plt.ylabel('Altitude '+r'$(m)$')

plt.subplot2grid((15,2),(0,1), rowspan=5)
plt.plot(m.time, m.w_x.value, label=r'$\omega_x$')
plt.plot(m.time, m.w_y.value, label=r'$\omega_y$')
plt.legend(loc='best')
plt.ylabel('Rotational velocity '+ r'$(\frac{rotations}{sec})$')

plt.subplot2grid((15,2),(3,0), rowspan=3)
plt.plot(m.time, m.vz.value, ':', color='darkorange', label='Model')
plt.plot(m.time, simZdot, '-', color='darkorange', label='Sim')
plt.legend()
plt.ylabel('Fall velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(5,1), rowspan=5)
plt.plot(m.time, m.θ_x.value, 'r:', label=r'$θ_x$ Model')
plt.plot(m.time, m.θ_y.value, 'b:', label=r'$θ_y$ Model')
plt.plot(m.time, simYaw, 'r-', label=r'$θ_x$ Sim')
plt.plot(m.time, simPitch, 'b-', label=r'$θ_y$ Sim')
plt.legend(loc='best')
plt.ylabel('Angle '+r'$(rad)$')

plt.subplot2grid((15,2),(6,0), rowspan=3)
plt.plot(m.time, m.vx.value, 'r:', label=r'$v_x$ Model')
plt.plot(m.time, m.vy.value, 'b:', label=r'$v_y$ Model')
plt.plot(m.time, simXdot, 'r-', label=r'$v_x$ Sim')
plt.plot(m.time, simYdot, 'b-', label=r'$v_y$ Sim')
plt.legend(loc='best')
plt.ylabel('Velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(9,0), rowspan=3)
plt.plot(m.time, m.x.value, 'r:', label='x Model')
plt.plot(m.time, m.y.value, 'b:', label='y Model')
plt.plot(m.time, simX, 'r-', label='x Sim')
plt.plot(m.time, simY, 'b-', label='y Sim')
plt.ylabel('Position '+r'$(m)$')
plt.legend(loc='best')

plt.subplot2grid((15,2),(10,1), rowspan=5)
plt.plot(m.time, m.Gimbalx.value, 'r--', label=r'$Gimbal_X$')
plt.plot(m.time, m.Gimbaly.value, 'b--', label=r'$Gimbal_Y$')
plt.legend(loc='best')
plt.ylabel('Gimbal '+r'$(rad)$')
plt.xlabel('Time')

plt.subplot2grid((15,2),(12,0), rowspan=3)
plt.plot(m.time, m.Thrust.value, 'k--')
plt.ylabel('Thrust')
plt.xlabel('Time')

plt.tight_layout()
plt.subplots_adjust(top=0.95,wspace=0.3)

plt.savefig(os.path.join('plots', simFilename + ' step test.png'))
plt.show()

