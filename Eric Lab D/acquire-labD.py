import numpy as np
import tclab
import time
import matplotlib.pyplot as plt

# Allow clean Ctrl-C shutdown
stop = False
import signal
def handler(signum, frame):
  global stop
  print ('Stopping')
  stop = True
signal.signal(signal.SIGINT, handler)

# Set up time grid
times = np.linspace(0, 600, 601)

# Set heater power
Q = np.zeros((2, times.size))
# Q[0,6:601] = 100.0   # Heater 1 on from 0.1 to 10 min
# Q[1,300:601] = 100.0  # Heater 2 on from 5 to 10 min

Q[0, 1:121] = 50.0
Q[0, 121:241] = 100.0
Q[0, 241:361] = 20.0
Q[0, 361:481] = 80.0
Q[0, 481:601] = 0.0
# Q[1, 1:61] = 20.0
# Q[1, 61:181] = 80.0
# Q[1, 181:301] = 0.0
# Q[1, 301:421] = 100.0
# Q[1, 421:541] = 50.0
# Q[1, 541:601] = 0.0


# Initialize measured temperature array
Tmeas = np.zeros((2, times.size))

# Connect to arduino
a = tclab.TCLab()

# Acquire temperatures once in case any other initialization is needed
a.T1
a.T2

# Set up timing loop
start_time = time.time()
sleep_interval = 1.0
sleep_until = start_time

# Enable/disable heater
enable_heater = True

try:
    
  for i in range(0, times.size):
    
    if stop:
      break

    # Read temperatures in K 
    Tmeas[0,i] = a.T1 + 273.15
    Tmeas[1,i] = a.T2 + 273.15

    if enable_heater:
      # Write output (0-100)
      a.Q1(Q[0,i])
      a.Q2(Q[1,i])

    print('{:6.2f} {:6.2f} {:6.2f} {:6.2f} {:6.2f}'.format(times[i], 
                                                    Q[0,i], 
                                                    Q[1,i], 
                                                    Tmeas[0,i], 
                                                    Tmeas[1,i]))

    # Sleep 
    sleep_until += sleep_interval
    sleep = sleep_until - time.time()
    if sleep > 0:
      time.sleep(sleep)
    else:
      print ('Warning: cycle time is too short')

finally:
  
  # Make sure we turn off the heaters and save the data in the event of an error
  print 'Disconnecting from arduino'
  a.Q1(0)
  a.Q2(0)
  time.sleep(1)   # Don't trust serial communication to be instantaneous
  a.close()

  # Save time, heater outputs, and measured temperatures
  data = np.vstack((times, Q[0,:], Q[1,:], Tmeas[0,:], Tmeas[1,:])).transpose()
  top = 'Time (sec), Heater 1 (%), Heater 2 (%), Temperature 1 (degC), Temperature 2 (degC)'
  np.savetxt('data_labD_arduino-with-bowl.txt', data, delimiter=',', header=top, comments='')

  plt.figure(figsize=(11,8))
  plt.subplot(2, 1, 1)
  plt.plot(times, Tmeas[0,:])
  plt.plot(times, Tmeas[1,:])

  plt.subplot(2, 1, 2)
  plt.plot(times, Q[0,:])
  plt.plot(times, Q[1,:])

  plt.show()