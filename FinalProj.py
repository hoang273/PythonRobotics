import PathPlanning.AStar.a_star as ast
import PathPlanning.BidirectionalAStar.bidirectional_a_star as bdas
import PathPlanning.BreadthFirstSearch.breadth_first_search as bfs
import PathPlanning.BidirectionalBreadthFirstSearch.bidirectional_breadth_first_search as bdbfs
import PathPlanning.DepthFirstSearch.depth_first_search as dfs
import PathPlanning.Dijkstra.dijkstra as djka
import PathPlanning.DStar.dstar as dst
import PathPlanning.GreedyBestFirstSearch.greedy_best_first_search as gbfs
import time
import math

import matplotlib.pyplot as plt
show_animation = True
def main():
    print(__file__ + " start!!")

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

    fig, axs = plt.subplots(1, 7)



    alogrithms=(
    ast.AStarPlanner(ox, oy, grid_size, robot_radius,axs[0]),
    dfs.DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius,axs[1]),
    bdas.BidirectionalAStarPlanner(ox, oy, grid_size, robot_radius,axs[2]),
    bdbfs.BidirectionalBreadthFirstSearchPlanner(ox, oy, grid_size, robot_radius,axs[3]),
    bfs.BreadthFirstSearchPlanner(ox, oy, grid_size, robot_radius,axs[4]),
    djka.Dijkstra(ox, oy, grid_size, robot_radius,axs[5]),
    gbfs.BestFirstSearchPlanner(ox, oy, grid_size, robot_radius,axs[6]),
    )
    timeElapsed=[]
    for count,alo in enumerate(alogrithms):
        startTime = time.time()
        rx,ry= alo.planning(sx, sy, gx, gy)
        endTime = time.time()
        timeRun=endTime-startTime
        timeElapsed.append(str(timeRun))
        axs[count].plot(rx, ry, "-r")

    if show_animation:  # pragma: no cover
        
        for i in range(len(alogrithms)):
            print(i)
            axs[i].plot(ox, oy, ".k")
            axs[i].plot(sx, sy, "og")
            axs[i].plot(gx, gy, "xb")
            axs[i].grid(True)
            axs[i].grid(True)
            axs[i].set_aspect('equal', 'box')
        plt.pause(.0001)
        plt.show()
    print(timeElapsed)
if __name__ == '__main__':
    main()
