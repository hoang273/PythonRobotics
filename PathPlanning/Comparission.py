from PathPlanning import *
import math
import matplotlib.pyplot as plt

print(__file__ + " start!!")
show_animation = True
# start and goal position
sx = 10.0  # [m]
sy = 10.0  # [m]
gx = 50.0  # [m]
gy = 50.0  # [m]
grid_size = 2.0  # [m]
robot_radius = 1.0  # [m]

# set obstacle positions
ox, oy = [], []
for i in range(-10, 60):
    ox.append(i)
    oy.append(-10.0)
for i in range(-10, 60):
    ox.append(60.0)
    oy.append(i)
for i in range(-10, 61):
    ox.append(i)
    oy.append(60.0)
for i in range(-10, 61):
    ox.append(-10.0)
    oy.append(i)
for i in range(-10, 40):
    ox.append(20.0)
    oy.append(i)
for i in range(0, 40):
    ox.append(40.0)
    oy.append(60.0 - i)

if show_animation:  # pragma: no cover
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
rx, ry = a_star.planning(sx, sy, gx, gy)

if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()