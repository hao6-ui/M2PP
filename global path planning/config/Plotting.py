import matplotlib.pyplot as plt
import matplotlib.patches as patches
from config.Utils import Utils



class Plotting:
    def __init__(self, x_start, x_goal,env):
        self.xI, self.xG = x_start, x_goal
        self.env = env
        self.delta= Utils(env).get_delta()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, smoothPath, smoothPath2, name, animation=False):
        self.plot_grid(name)
        # plt.legend()
        # self.plot_visited(nodelist, animation)
        self.plot_path(path)
        # self.plot_Smooth_Path(smoothPath)
        # plt.legend()
        plt.show()
        # self.plot_Smooth_Path2(smoothPath2)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox-self.delta, oy-self.delta), w+2*self.delta, h+2*self.delta,
                    edgecolor='black',
                    facecolor='white',
                    fill=True
                )
            )
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r+self.delta,
                    edgecolor='black',
                    facecolor='white',
                    fill=True
                )
            )
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )


        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',     # mouse ticking stop
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([node.x for node in path], [node.y for node in path], '-r', linewidth=2, label="Original path")
            plt.pause(0.01)
        #plt.show()

    @staticmethod
    def plot_Smooth_Path(smoothPath):
        plt.plot(smoothPath.T[0], smoothPath.T[1], 'b', linewidth=1.5, label="Bezier smoothing path")

    @staticmethod
    def plot_Smooth_Path2(smoothPath):
        plt.plot(smoothPath.T[0], smoothPath.T[1], 'k', linewidth=1.5, label="B-spline")
        plt.legend()
        plt.show()


