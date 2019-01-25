import numpy as np

def euler(dy, y0, time):
  y = y0
  t1 = time[0]
  soln = [y0]
  for t2 in time[1:]:
    dt = t2-t1
    _dy = dy(time, y)
    y = y + _dy * dt
    soln.append(y)
    t1 = t2
  
  return soln


def rk4(dy, y0, time):
  y = y0
  t1 = time[0]
  soln = [y0]
  for t2 in time[1:]:
    dt = t2-t1
    k1 = dy(time, y) * dt
    k2 = dy(time + dt*0.5, y + k1*0.5) * dt
    k3 = dy(time + dt*0.5, y + k2*0.5) * dt
    k4 = dy(time + dt, y + k3) * dt

    y = y + (k1 + 2*k2 + 2*k3 + k4) / 6
    soln.append(y)
    t1 = t2
  
  return soln