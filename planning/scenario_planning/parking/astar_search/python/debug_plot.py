import matplotlib.pyplot as plt
import numpy as np
import rosbag

class CarModel(object):
    def __init__(self, length=5.5, width=2.75, base2back=1.5):
        self.length = length
        self.width = width
        self.base2back = base2back

    def _get_four_points(self):
        back = -1.0 * self.base2back
        front = self.length - self.base2back
        right = - 0.5 * self.width
        left = 0.5 * self.width
        P = np.array([[back, left], [back, right], [front, right], [front, left]])
        return P

    def get_four_points(self, pos):
        pos = np.array(pos)
        P_ = self._get_four_points()
        P = P_ + pos[None, :]
        return P

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

    def plot(self, pose_start, pose_goal):
        fig, ax = plt.subplots()
        xlin, ylin = [np.linspace(self.b_min[i], self.b_max[i], self.n_grid[i]) for i in range(2)]
        X, Y = np.meshgrid(xlin, ylin)
        ax.contourf(X, Y, self.arr)

        car = CarModel()

        def plot_pose(pose):
            pos_xy = pose[:2]
            P = car.get_four_points(pos_xy)
            for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
                i, j = idx_pair
                ax.plot([P[i, 0], P[j, 0]], [P[i, 1], P[j, 1]], color="red")


        plot_pose(pose_start)
        plot_pose(pose_goal)
        ax.axis("equal")
        plt.show()

bag = rosbag.Bag("../bag/astar.bag")
for topic, msg, _ in bag.read_messages(topics=["costmap"]):
    costmap_msg = msg
costmap = Costmap(costmap_msg)
pose_start = [3714.06, 73750.3, 0.0541427, 8.21603e-05, 5.30234e-05, 0.228649, 0.973509]
pose_goal = [3719.11, 73745.8, 1.00101, 0, 0, 0.853535, 0.521036]
costmap.plot(pose_start, pose_goal)

