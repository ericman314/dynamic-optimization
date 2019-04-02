import numpy as np
import matplotlib.pyplot as plt
import os

from model import getModel

# Filename to read initial conditions from (don't include the .csv)
#initFilename = '50m-drop'
initFilename = '40km-5%prop-750mpsdown'

# Filename to read step tests from (don't include the .csv)
stepFilename = 'engineOn'


# Create the model
m = getModel(initFilename)



m.options.IMODE = 4  # Just simulation for now, but the ultimate plan is for this to control
n = 60
m.time = np.linspace(0, 60, n)

# Time (sec), Throttle (0-1), EngineOn (0 or 1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg)
stepTests = np.loadtxt(os.path.join('stepTests', stepFilename + '.csv'), delimiter=',')

# The step test file has one row for every step change, but we need one row for every timestep.
# Create arrays to hold the MV values for each timestep.
stepThrottle = np.zeros(m.time.size)
stepEngineOn = np.zeros(m.time.size)
stepGimbalX = np.zeros(m.time.size)
stepGimbalY = np.zeros(m.time.size)
stepGridX = np.zeros(m.time.size)
stepGridY = np.zeros(m.time.size)

# Assign the step test values from the file to our per-timestep arrays
i = -1
j = -1
for t in m.time:
  i += 1
  # Find the right time

  while j+1 < stepTests.shape[0] and stepTests[j+1][0] < t:
    j += 1
  print (i, j, stepTests[j+1][0], t)
  stepThrottle[i] = stepTests[j, 1]
  stepEngineOn[i] = stepTests[j, 2]
  stepGimbalX[i] = stepTests[j, 3]
  stepGimbalY[i] = stepTests[j, 4]
  stepGridX[i] = stepTests[j, 5]
  stepGridY[i] = stepTests[j, 6]

# Assign step test values to the model
m.Thrust.value = stepThrottle
#m.EngineOn.value = stepEngineOn
m.Gimbalx.value = stepGimbalX
m.Gimbaly.value = stepGimbalY
#m.Gridx.value = stepGridX
#m.Gridy.value = stepGridY

m.solve(Remote=False)

plt.figure(num=2, figsize=(10,8))
plt.subplot2grid((15,2),(0,0), rowspan=3)
plt.plot(m.time, m.z.value)
plt.ylabel('Altitude '+r'$(m)$')

plt.subplot2grid((15,2),(0,1), rowspan=5)
plt.plot(m.time, m.w_x.value, label=r'$\omega_x$')
plt.plot(m.time, m.w_y.value, label=r'$\omega_y$')
plt.legend(loc='best')
plt.ylabel('Rotational velocity '+ r'$(\frac{rotations}{sec})$')

plt.subplot2grid((15,2),(3,0), rowspan=3)
plt.plot(m.time, m.vz.value)
plt.ylabel('Fall velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(5,1), rowspan=5)
plt.plot(m.time, m.θ_x.value, 'r', label=r'$θ_x$')
plt.plot(m.time, m.θ_y.value, 'b', label=r'$θ_y$')
plt.legend(loc='best')
plt.ylabel('Angle '+r'$(rad)$')

plt.subplot2grid((15,2),(6,0), rowspan=3)
plt.plot(m.time, m.vx.value, 'r', label=r'$v_x$')
plt.plot(m.time, m.vy.value, 'b', label=r'$v_y$')
plt.legend(loc='best')
plt.ylabel('Velocity '+r'$(\frac{m}{s})$')

plt.subplot2grid((15,2),(9,0), rowspan=3)
plt.plot(m.time, m.x.value, 'r', label='x')
plt.plot(m.time, m.y.value, 'b', label='y')
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
plt.show()
