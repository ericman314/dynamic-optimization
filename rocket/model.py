from time import sleep


def runController(x, y, z, yaw, pitch, prop, time):

  # Using x, y, z, yaw, pitch, and prop as controlled variables, optimize gridX, gridY, gimbalX, gimbalY, throttle, and engineOn to meet the objective

  # Perform a step test or something


  gridX = 0
  gridY = 0
  gimbalX = 0
  gimbalY = 0
  throttle = 0
  engineOn = False

  # Step test on gridX
  if 10 < time < 20:
    gridX = -10
  if 20 < time < 30:
    gridX = 10

  # There is not yet any mechanism in simulation.py that limits how often the controller runs, so do this otherwise the controller will run way faster than it needs to
  sleep(1)

  return { 'gridX': gridX,
           'gridY': gridY,
           'gimbalX': gimbalX,
           'gimbalY': gimbalY,
           'throttle': throttle,
           'engineOn': engineOn
         }