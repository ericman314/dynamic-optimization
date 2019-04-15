from model import getModel
import numpy as np
import matplotlib.pyplot as plt
import os.path

controllerOutputFilename = 'offlineController output.csv'
simulationOutputFilename = os.path.join('simulationData', 'nrol-76_offlineController-output.csv')

# Time (sec), X (m), Y (m), Z (m), Xdot (m/s), Ydot (m/s), Zdot (m/s), PropMass (kg), Throttle (0-1), Yaw (deg), Pitch (deg)
ctrlTime, ctrlX, ctrlY, ctrlZ, ctrlVx, ctrlVy, ctrlVz, ctrlProp, ctrlThrottle, ctrlYaw, ctrlPitch = \
  np.loadtxt(controllerOutputFilename, delimiter=',', unpack=True)

# Time (sec), X (m), Y (m), Z (m), Roll (deg), Yaw (deg), Pitch (deg), Xdot (m/s), Ydot (m/s), Zdot (m/s), PropMass (kg), Throttle (0-1), GimbalX (deg), GimbalY (deg), GridX (deg), GridY (deg), GeeAxial (g), GeeLateral (g), AOA (deg), MvYaw (deg), MvPitch (deg)
simTime, simX, simY, simZ, simRoll, simYaw, simPitch, simVx, simVy, simVz, simProp, simThrottle, simGimbalx, simGimbaly, simGridx, simGridy, simGeeAxial, simGeeLateral, simAOA, simMvYaw, simMvPitch = \
  np.loadtxt(simulationOutputFilename, delimiter=',', unpack=True)



plt.figure(figsize=(11,8))
plt.subplot(2, 3, 1)
plt.plot(ctrlTime, ctrlZ, '--')
plt.plot(simTime, simZ, '-', label='z')
plt.legend()

plt.subplot(2, 3, 2)
plt.plot(ctrlTime, ctrlVz, '--')
plt.plot(simTime, simVz, '-', label='vz')
plt.legend()

# plt.subplot(2, 3, 3)
# plt.plot(ctrltime, ctrlThrottle, label='Throttle')
# plt.plot(ctrltime, ctrlEngineOn, label='EngineOn')
# plt.legend()
# plt.subplot(2, 3, 4)

plt.plot(ctrlTime, ctrlX, 'r--')
plt.plot(simTime, simX, 'b-', label='x')
plt.plot(ctrlTime, ctrlY, 'r--')
plt.plot(simTime, simY, 'b-', label='y')
plt.legend()
plt.subplot(2, 3, 5)
plt.plot(ctrlTime, ctrlVx, 'r--')
plt.plot(simTime, simVx, 'r-', label='vx')
plt.plot(ctrlTime, ctrlVy, 'b--')
plt.plot(simTime, simVy, 'b-', label='vy')
plt.legend()

# plt.subplot(2, 3, 6)
# plt.plot(ctrltime, ctrlYaw, 'r-', label='Yaw')
# plt.plot(ctrltime, ctrlPitch, 'b-', label='Pitch')
# plt.legend()

plt.show()

