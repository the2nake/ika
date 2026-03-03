import matplotlib.pyplot as plt
import math

base = (0, 0, 0)
lower_yaw = math.pi / 4
lower_pitch = math.pi / 4
lower_length = 10

distance = lower_length * math.cos(lower_pitch)
elbow = (distance * math.cos(lower_yaw), distance * math.sin(lower_yaw), base[2] + lower_length * math.sin(lower_pitch))


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

xs = [base[0], elbow[0]]
ys = [base[1], elbow[1]]
zs = [base[2], elbow[2]]

ax.plot3D(xs, ys, zs)

ax.set_xlim3d(-20, 20)
ax.set_ylim3d(-20, 20)
ax.set_zlim3d(0, 30)
plt.show()