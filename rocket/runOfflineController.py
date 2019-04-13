from model import getModel
import numpy as np
import matplotlib.pyplot as plt
import os.path

initData = np.loadtxt('offlineController init.csv', delimiter=',').transpose()

mpc = getModel(name='mpc-simple')
mpc.time = np.linspace(0, 70, 36)


_EngineOn = np.zeros(36)
_EngineOn[22:] = 1
mpc.EngineOn.VALUE = _EngineOn

# mpc.x.VALUE = initData[1,:]
# mpc.y.VALUE = initData[2,:]
# mpc.z.VALUE = initData[3,:]
# mpc.vx.VALUE = initData[4,:]
# mpc.vy.VALUE = initData[5,:]
# mpc.vz.VALUE = initData[6,:]
# mpc.propMass.VALUE = initData[7,:]
# mpc.Throttle.VALUE = initData[8,:]
# mpc.Yaw.VALUE = initData[9,:]
# mpc.Pitch.VALUE = initData[10,:]


mpc.x.VALUE = initData[1,0]
mpc.y.VALUE = initData[2,0]
mpc.z.VALUE = initData[3,0]
mpc.vx.VALUE = initData[4,0]
mpc.vy.VALUE = initData[5,0]
mpc.vz.VALUE = initData[6,0]
mpc.propMass.VALUE = initData[7,0]
mpc.Throttle.VALUE = initData[8,0]
mpc.Yaw.VALUE = initData[9,0]
mpc.Pitch.VALUE = initData[10,0]



mpc.options.IMODE = 6
mpc.options.MAX_ITER = 500

mpc.x.FSTATUS = 1
mpc.y.FSTATUS = 1
mpc.z.FSTATUS = 1
mpc.vx.FSTATUS = 1
mpc.vy.FSTATUS = 1
mpc.vz.FSTATUS = 1
mpc.propMass.FSTATUS = 1


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
mpc.Obj( (mpc.vx**2 + mpc.x**2) * finalMask  )
mpc.Obj( (mpc.vy**2 + mpc.y**2) * finalMask  )

# mpc.Obj( mpc.x**2 + mpc.y**2 )

# mpc.Obj( xCrash**2 + yCrash**2 )

# Maybe we'll dry a different objective that keeps the rocket on a ballistic trajectory towards the origin, instead of just the final point.

mpc.options.RTOL = 1e0
mpc.options.OTOL = 1e-2

mpc.options.COLDSTART = 1

mpc.solve()



mpc.solve()

mpc.solve()

# Time (sec), X (m), Y (m), Z (m), Xdot (m/s), Ydot (m/s), Zdot (m/s), PropMass (kg), Throttle (0-1), Yaw (deg), Pitch (deg)
data = np.vstack((
  mpc.time,
  mpc.x,
  mpc.y,
  mpc.z,
  mpc.vx,
  mpc.vy,
  mpc.vz,
  mpc.propMass,
  mpc.Throttle,
  mpc.Yaw,
  mpc.Pitch
)).transpose()

top = '# Time (sec), X (m), Y (m), Z (m), Xdot (m/s), Ydot (m/s), Zdot (m/s), PropMass (kg), Throttle (0-1), Yaw (deg), Pitch (deg)'
np.savetxt('offlineController output.csv', data, fmt='%.4f', delimiter=', ', header=top)


mpc.EngineOn[-1] = 0
stepTest = np.vstack((
  mpc.time,
  mpc.Throttle,
  mpc.EngineOn,
  mpc.Yaw,
  mpc.Pitch
)).transpose()

top = '# Time (sec), Throttle (0-1), EngineOn(0, 1), Yaw (deg), Pitch (deg)'
np.savetxt(os.path.join('stepTests', 'offlineController-output.csv'), stepTest, fmt='%.4f', delimiter=', ', header=top)


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

plt.savefig('offlineController output.png')
plt.show()

