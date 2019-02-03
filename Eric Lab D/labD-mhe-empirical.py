import numpy as np
from gekko import GEKKO
import matplotlib.pyplot as plt

# Model
m = GEKKO(remote=False)
m.options.IMODE = 5 # MHE
m.options.EV_TYPE = 1 # l1-norm

horizon = 60 # horizon window
m.time = np.linspace(0, horizon, horizon+1)

# Model parameters
m.K = m.FV(value=1, lb=0.01, ub=100)
m.tau1 = m.FV(value=174, lb=1, ub=1000)
m.tau2 = m.FV(value=16, lb=1, ub=100)

m.K.STATUS = 1
m.tau1.STATUS = 0
m.tau2.STATUS = 0
m.K.FSTATUS = 0
m.tau1.FSTATUS = 0
m.tau2.FSTATUS = 0

m.K.DMAX = 0.005
m.tau1.DMAX = 0.5
m.tau2.DMAX = 0.01

# Ambient temperature
_Ta = 295
m.Ta = m.FV(value = _Ta)

# Manipulated variable (process input)
m.Q1 = m.MV()
m.Q1.FSTATUS = 1

# State Variables
m.T1heater = m.SV(value = _Ta)

# Measured variables

m.T1sensor = m.CV(value = _Ta)
m.T1sensor.FSTATUS = 1
m.T1sensor.MEAS_GAP = 0
m.T1sensor.TR_INIT = 1



# Equations
m.Equation( m.tau1 * m.T1heater.dt() == m.K * m.Q1 - (m.T1heater - m.Ta) )
m.Equation( m.tau2 * m.T1sensor.dt() == (m.T1heater - m.T1sensor) )

# Load measured data
data = np.loadtxt('data_labD_arduino-with-bowl.txt', delimiter=',', skiprows=1)
data_time = data[:,0]
data_Q1 = data[:,1]
data_T1 = data[:,3]
cycles = data_time.size

# Add normally distributed noise to the data
for i in range(cycles):
  data_T1[i] += np.random.normal(0, 5.0)

# Run the estimator
K_est = np.ones(cycles) * m.K.value
tau1_est = np.ones(cycles) * m.tau1.value
tau2_est = np.ones(cycles) * m.tau2.value
T1_est = np.ones(cycles) * m.T1sensor.value

# Set up plot
# plt.ion()
fig = plt.figure(figsize=(11,8))
nr = 5
nc = 1
sp1 = fig.add_subplot(nr,nc,1)
sp2 = fig.add_subplot(nr,nc,2)
sp3 = fig.add_subplot(nr,nc,3)
sp4 = fig.add_subplot(nr,nc,4)
sp5 = fig.add_subplot(nr,nc,5)

sp1.plot(data_time, data_T1, '*')
line1, = sp1.plot(data_time, T1_est, '-')

line2, = sp2.plot(data_time, K_est, '-')

line3, = sp3.plot(data_time, tau1_est, '-')
line4, = sp4.plot(data_time, tau2_est, '-')

sp5.plot(data_time, data_Q1, '-')

fig.canvas.draw()
plt.show(block=False)
for i in range(cycles):
  m.Q1.MEAS = data_Q1[i]
  m.T1sensor.MEAS = data_T1[i]

  try:
    m.solve(disp=False)
    K_est[i] = m.K.NEWVAL
    tau1_est[i] = m.tau1.NEWVAL
    tau2_est[i] = m.tau2.NEWVAL
    T1_est[i] = m.T1sensor.MODEL
  except:
    print 'Iteration ', i, ' failed'
    pass

  if i%20 == 0:
    line1.set_ydata(T1_est)
    line2.set_ydata(K_est)
    line3.set_ydata(tau1_est)
    line4.set_ydata(tau2_est)
    sp1.relim()
    sp2.relim()
    sp3.relim()
    sp4.relim()
    sp1.autoscale_view(True, True, True)
    sp2.autoscale_view(True, True, True)
    sp3.autoscale_view(True, True, True)
    sp4.autoscale_view(True, True, True)

    fig.canvas.draw()
    print m.K.value[0], m.tau1.value[0], m.tau2.value[0] 

line1.set_ydata(T1_est)
line2.set_ydata(K_est)
line3.set_ydata(tau1_est)
line4.set_ydata(tau2_est)
fig.canvas.draw()


plt.show()