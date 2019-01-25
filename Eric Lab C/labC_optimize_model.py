import numpy as np
import matplotlib.pyplot as plt
import labC_model

# Filename without '.txt'
fn = 'data_labC_arduino1'
# fn = 'data_labB_arduino'
# fn = 'data_labB_alt_arduino'

# Load previously acquired data and heater setpoints
data = np.loadtxt(fn + '.txt', skiprows=1, delimiter=',')
times = data[:,0]
_Q1 = data[:,1]
_Q2 = data[:,2]
T1meas = data[:,3]
T2meas = data[:,4]


# Run the dynamic model
m = labC_model.getModel(times, _Q1, _Q2, T1meas, T2meas)
m.solve()

print m.alpha1.value[0]
print m.alpha2.value[0]
print m.Ta.value[0]
print m.U.value[0]
print m.As.value[0]
print m.cond1.value[0]
print m.cond2.value[0]

# Plot results
plt.figure(figsize=(10,4))
plt.title('Adding time delay')
plt.plot(times/60, T1meas-273.15, 'r-', label='T1 measured')
plt.plot(times/60, T2meas-273.15, 'b-', label='T2 measured')
plt.plot(times/60, np.array(m.T1.value)-273.15, 'r--', label='T1 model')
plt.plot(times/60, np.array(m.T2.value)-273.15, 'b--', label='T2 model')
# plt.plot(times/60, np.array(m.Tint1.value)-273.15, 'r.-', label='T1 int')
# plt.plot(times/60, np.array(m.Tint2.value)-273.15, 'b.-', label='T2 int')
plt.ylabel('Temperature (degC)')
plt.xlabel('Time (min)')
plt.legend()



plt.savefig(fn + '.time-delay.png')

plt.show()

# plt.figure(figsize=(10,4))
# plt.title('Heater Outputs')
# plt.plot(times/60, _Q1, 'r-', label='Heater 1')
# plt.plot(times/60, _Q2, 'b-', label='Heater 2')
# plt.ylabel('Power (%)')
# plt.xlabel('Time (min)')
# plt.legend()

# plt.show()