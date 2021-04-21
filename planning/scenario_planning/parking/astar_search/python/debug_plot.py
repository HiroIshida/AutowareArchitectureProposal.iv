import matplotlib.pyplot as plt
import numpy as np
import rosbag

class Costmap(object):
    def __init__(self, msg):
        info = msg.info
        n_grid = np.array([info.width, info.height])
        res = info.resolution
        origin = info.origin

        tmp = np.array(msg.data).reshape((n_grid[1], n_grid[0])) # about to be transposed!!

        self.arr = tmp # [IMPORTANT] !!
        self.b_min = np.array([origin.position.x, origin.position.y])
        self.b_max = self.b_min + n_grid * res
        self.n_grid = n_grid
        self.origin = origin

    def plot(self, pos_start, pos_goal):
        fig, ax = plt.subplots()
        xlin, ylin = [np.linspace(self.b_min[i], self.b_max[i], self.n_grid[i]) for i in range(2)]
        X, Y = np.meshgrid(xlin, ylin)
        ax.contourf(X, Y, self.arr)
        ax.scatter(pos_start[0], pos_start[1])
        ax.scatter(pos_goal[0], pos_goal[1])
        ax.axis("equal")
        plt.show()

bag = rosbag.Bag("../bag/astar.bag")
for topic, msg, _ in bag.read_messages(topics=["costmap"]):
    costmap_msg = msg
costmap = Costmap(costmap_msg)
costmap.plot([3714.06, 73750.3], [3719.11, 73745.8])

