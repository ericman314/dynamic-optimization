from model import getModel
import numpy as np
import matplotlib.pyplot as plt

v = [ 6.17830566e+03,  3.50692603e+03,  3.37449648e+04, -1.48535400e+02,
 -8.92251434e+01, -7.96663025e+02,  1.83035949e-01, 1.13647645e-01,
  1.99493699e-06,  4.48860824e-06,  2.08500000e+04]

mpc = getModel(name='mpc-simple')

mpc.options.IMODE = 6
mpc.options.MAX_ITER = 500

mpc.x.FSTATUS = 1
mpc.y.FSTATUS = 1
mpc.z.FSTATUS = 1
mpc.vx.FSTATUS = 1
mpc.vy.FSTATUS = 1
mpc.vz.FSTATUS = 1
mpc.propMass.FSTATUS = 1

mpc.time = np.linspace(0, 70, 71)

# Set initial conditions
mpc.x.VALUE = v[0]
mpc.y.VALUE = v[1]
mpc.z.VALUE = v[2]
mpc.vx.VALUE = v[3]
mpc.vy.VALUE = v[4]
mpc.vz.VALUE = v[5]
mpc.propMass.VALUE = v[10]

# mpc.EngineOn.STATUS = 1
mpc.Throttle.STATUS = 1
mpc.Yaw.STATUS = 1
mpc.Pitch.STATUS = 1
mpc.Yaw.DMAX = 1
mpc.Pitch.DMAX = 1

_finalMask = np.zeros(mpc.time.size)
_finalMask[-1] = 1
finalMask = mpc.Param(_finalMask)

# tCrash = mpc.Intermediate((mpc.vz + mpc.sqrt(mpc.vz**2 + 2*mpc.g*mpc.z)) / mpc.g)
# xCrash = mpc.Intermediate(mpc.x + mpc.vx * tCrash)
# yCrash = mpc.Intermediate(mpc.y + mpc.vy * tCrash)

mpc.Obj( (mpc.vz**2 + mpc.z**2) * finalMask  )
mpc.Obj( mpc.x**2 * finalMask  )
mpc.Obj( mpc.y**2 * finalMask  )

# Maybe we'll dry a different objective that keeps the rocket on a ballistic trajectory towards the origin, instead of just the final point.

mpc.options.RTOL = 1e0
mpc.options.OTOL = 1e-2

mpc.options.COLDSTART = 1

mpc.solve()


mpc.solve()


plt.figure(figsize=(11,8))
plt.subplot(2, 3, 1)
plt.plot(mpc.time, mpc.z.value, label='z')
plt.legend()
plt.subplot(2, 3, 2)
plt.plot(mpc.time, mpc.vz.value, label='vz')
plt.legend()
plt.subplot(2, 3, 3)
plt.plot(mpc.time, mpc.Throttle, label='Throttle')
plt.plot(mpc.time, mpc.EngineOn, label='EngineOn')
plt.legend()
plt.subplot(2, 3, 4)
plt.plot(mpc.time, mpc.x, 'r-', label='x')
plt.plot(mpc.time, mpc.y, 'b-', label='y')
plt.legend()
plt.subplot(2, 3, 5)
plt.plot(mpc.time, mpc.vx, 'r-', label='vx')
plt.plot(mpc.time, mpc.vy, 'b-', label='vy')
plt.legend()
plt.subplot(2, 3, 6)
plt.plot(mpc.time, mpc.Yaw, 'r-', label='Yaw')
plt.plot(mpc.time, mpc.Pitch, 'b-', label='Pitch')
plt.legend()

plt.show()