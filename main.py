import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import cProfile

# create swift instance
env = swift.Swift()
env.launch(realtime=True)

# Initialise model
panda = rtb.models.Panda()
panda.q = panda.qr

panda.qd = [0.1,0,0,0,0,0,0.1]

# print the urdf string of the panda
print(panda.urdf_string)

# add panda to swift
env.add(panda, robot_alpha=0.5)

# set goal pose
tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.35)

axes = sg.Axes(length=0.1, base=tep)
env.add(axes)

# Arrived at destination flag
arrived = False

# Time step
dt = 0.01#


while not arrived:

    # v is a 6 vector representing the spatial error
    v, arrived = rtb.p_servo(panda.fkine(panda.q), tep, gain=1, threshold=0.01)

    J = panda.jacobe(panda.q)

    panda.qd = np.linalg.pinv(J) @ v

    # step the environment
    env.step(dt)

# stop the browser from closing
env.hold()