from gekko import GEKKO
import numpy as np

m = GEKKO(remote=False)
m.options.IMODE = 5
m.options.NODES = 3

x = m.CV(0)

vx = m.CV()

m.Equation(x.dt() == vx)
m.Equation(vx.dt() == 1)

x.FSTATUS = 1
# vx.STATUS = 1

m.time = np.linspace(0, 4, 5)

m.options.ICD_CALC = 1

# Time 0
x.MEAS = 10
m.solve()

x.MEAS = 11
m.solve()

x.MEAS = 14
m.solve()

x.MEAS = 16
m.solve()

x.MEAS = 20
m.solve()

print (x.value)
print (vx.value)
