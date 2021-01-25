import time
from robolimb import RoboLimbCAN as RL

r = RL()
r.start()
r.close_finger(1)
r.close_finger(2)
r.close_finger(3)
r.close_finger(4)
r.close_finger(5)
time.sleep(5)
r.open_finger(1)
r.open_finger(2)
r.open_finger(3)
r.open_finger(4)
r.open_finger(5)

time.sleep(5)
r.stop()