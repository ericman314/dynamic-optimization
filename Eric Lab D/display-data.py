import numpy as np
import matplotlib.pyplot as plt

data0 = np.loadtxt('data_labD_arduino-without-bowl.txt', delimiter=',', skiprows=1)
data1 = np.loadtxt('data_labD_arduino-with-bowl.txt', delimiter=',', skiprows=1)

# Set up time grid
times = np.linspace(0, 600, 601)

plt.figure(figsize=(11,8))
plt.subplot(2, 1, 1)
plt.plot(data0[:,0], data0[:,3])
plt.plot(data0[:,0], data0[:,4])

plt.plot(data1[:,0], data1[:,3])
plt.plot(data1[:,0], data1[:,4])


plt.subplot(2, 1, 2)
plt.plot(data0[:,0], data0[:,1])
plt.plot(data0[:,0], data0[:,2])

plt.show()