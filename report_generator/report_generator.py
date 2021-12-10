import glob
import argparse
import sys, os, os.path
import numpy as np
import json
from math import *
import matplotlib.patches as mpatches
import tools.transformations as tf
import tools.file_loader as fl
from tools.data_linewidth_plot import *

sys.path.insert(1, '/home/xavier/openvslam/report_generator')

class ReportGenerator(object):
    def __init__(self, config):
        super(ReportGenerator).__init__()

        # parse json config file
        if config.config != '':
            f = open(config.config)
            data = json.load(f)

            dirs = data['Directories']
            if config.traj_dir is None:
                config.traj_dir = dirs['TrajectoryDir']
            if config.map_dir is None:
                config.map_dir = dirs['MapDir']
            if config.result_dir is None:
                config.result_dir = dirs['ResultDir']

            params = data['Parameters']
            if config.floor == -999:
                config.floor = params['FloorHeight']
            if config.ceiling == -999:
                config.ceiling = params['CeilingHeight']
            if config.brush_width == -999:
                config.brush_width = params['BrushWidth']

        assert os.path.exists(config.traj_dir), "Trajectorie path doesn't exist! path: {}".format(config.traj_dir)
        assert os.path.exists(config.map_dir), "Map path doesn't exist! path: {}".format(config.map_dir)

        map_files = glob.glob(config.map_dir + '/*.msg')
        if len(map_files) > 1:
            print('Too many map files in the map folder! Should only be 1!')
            exit(1)
        elif len(map_files) == 0:
            print('No map files found in the map directory {}!'.format(config.map_dir))
            exit(1)
        self.map_file = map_files[0]

        map_trajectories = glob.glob(config.map_dir + '/*.txt')
        if len(map_trajectories) > 1:
            print('Too many map trajectories in the map directory {}! Should only be 1!'.format(config.map_dir))
            exit(1)
        elif len(map_trajectories) == 0:
            print('No trajectory files found in the map directory {}!'.format(config.map_dir))
            exit(1)
        self.map_trajectory = map_trajectories[0]

        traj_files = glob.glob(config.traj_dir + '/*.txt')
        traj_files = [os.path.split(f)[1] for f in traj_files]
        if len(traj_files) == 0:
            print("No trajectories found in trajectory path {}!".format(config.traj_dir))
            exit(1)

        self.trajectories = [os.path.splitext(f)[0] for f in traj_files]
        self.traj_dir = config.traj_dir
        self.result_dir = config.result_dir

        self.ceiling = config.ceiling
        self.floor = config.floor
        self.brush_width = config.brush_width

        if not os.path.exists(self.result_dir):
            os.makedirs(self.result_dir)

    def get_map_gradiant(self, map_points):
        """
        Returns a list of floats from 0 to 1. The floats are used in combination with a cmap to assign a color
        to each individual map point depending on it\'s offset in the vertical direction.
        """
        max_y = max(map_points[:][1])
        min_y = fabs(min(map_points[:][1]))
        delta_y = max_y + min_y

        map_gradiant = []

        for i in range(len(map_points)):
            point = map_points[i]
            point[1] = (point[1] + min_y) / delta_y
            map_gradiant.append(point[1])

        return map_gradiant

    def get_point_colors(self, map_points):
        """
        Returns a list of floats from 0 to 1. The floats are used in combination with a cmap to assign a color
        to each individual map point depending on it\'s offset in the vertical direction. Unlike the gradiant there are
        3 set colors that are assigned
        :param map_points: matrix of map points
        :return: list of color values
        """
        color = []
        for point in map_points:
            if point[1] > 0.3:
                color.append(0)
            elif point[1] < 0:
                color.append(0.5)
            else:
                color.append(1)

        return color

    def plot_trajectories_3d(self, trajectories, map_points, map_traj, toCameraCoord):
        """
            Plots the trajectories in the 3D

            Params:
                trajectories - list of trajectory names
                trajectory_dir - directory that contains all the trajectories
                results_dir - directory that the results will be saved to
                map - point cloud of the enviorments
                toCameraCoord - do the poses need to be converted to the camera coordinate space
        """
        from mpl_toolkits.mplot3d import Axes3D  # is needed, don't delete

        font_size = 8
        start_point = [[0], [0], [0]]
        style_start_point = 'ko'
        style_map = '--r'
        style_dict = {
            0: "-b",
            1: "-g",
            2: "-y",
            3: "-m",
            4: "-c"
        }

        fig = plt.figure(figsize=(8, 8), dpi=110)
        ax = fig.gca(projection='3d')

        # plot each trajectory in the trajectory directory
        for i in range(len(trajectories)):
            poses = fl.load_poses("{}/{}.txt".format(self.traj_dir, trajectories[i]), toCameraCoord)
            poses = tf.rotate_trajectory(poses)
            x_traj = [pose[0] for pose in poses]
            y_traj = [pose[1] for pose in poses]
            z_traj = [pose[2] for pose in poses]

            plt.plot(x_traj, z_traj, y_traj, style_dict[i], label=trajectories[i])

        # plot the map trajectory
        # x_map_traj = [pose[0] for pose in map_traj]
        # y_map_traj = [pose[1] for pose in map_traj]
        # z_map_traj = [pose[2] for pose in map_traj]
        # plt.plot(x_map_traj, z_map_traj, y_map_traj, style_map, label='map_traj')

        # plot the start point
        plt.plot(start_point[0], start_point[1], start_point[2], style_start_point, label='Start Point')

        # set limits for the axis
        xlim = ax.get_xlim3d()
        ylim = ax.get_ylim3d()
        zlim = ax.get_zlim3d()
        xmean = np.mean(xlim)
        ymean = np.mean(ylim)
        zmean = np.mean(zlim)
        plot_radius = max([abs(lim - mean_)
                           for lims, mean_ in ((xlim, xmean),
                                               (ylim, ymean),
                                               (zlim, zmean))
                           for lim in lims]) + 3
        ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
        ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
        ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])

        # plot the map as a scatter plot of points
        x_map = [point[0] for point in map_points]
        y_map = [point[1] for point in map_points]
        z_map = [point[2] for point in map_points]

        # gets each points color according to its y value on the map
        # map_gradiant = self.get_map_gradiant(culled_map)
        point_color = self.get_point_colors(map_points)
        ax.scatter(x_map, z_map, y_map, s=0.3, c=point_color, cmap="Set1")

        ax.legend()
        ax.set_xlabel('x (m)', fontsize=font_size)
        ax.set_ylabel('z (m)', fontsize=font_size)
        ax.set_zlabel('y (m)', fontsize=font_size)
        ax.view_init(elev=90, azim=-90)

        plt.show()
        plt.close()

    def plot_trajectories_2d(self, trajectories, map_points, map_traj, toCameraCoord):
        """
            Plots the trajectories in the XZ plane and saves them to a png and pdf

            Params:
                trajectories - list of trajectory names
                trajectory_dir - directory that contains all the trajectories
                results_dir - directory that the results will be saved to
                map - point cloud of the enviorments
                toCameraCoord - do the poses need to be converted to the camera coordinate space
        """
        font_size = 10
        start_point = [0, 0]
        style_start_point = 'ko'
        style_map = '--r'
        style_dict = {
            0: "-b",
            1: "-g",
            2: "-y",
            3: "-m",
            4: "-c"
        }

        fig = plt.figure(figsize=(20, 6), dpi=100)
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        plt.xlabel('x (m)', fontsize=font_size)
        plt.ylabel('z (m)', fontsize=font_size)
        plt.grid()

        #  load each trajectory and plot it
        for i in range(len(trajectories)):
            poses = fl.load_poses("{}/{}.txt".format(self.traj_dir, trajectories[i]), toCameraCoord)
            poses = tf.rotate_trajectory(poses)
            x_traj = [pose[0] for pose in poses]
            z_traj = [pose[2] for pose in poses]

            data_linewidth_plot(x_traj, z_traj, linewidth=self.brush_width, c='gold', zorder=0)
            plt.plot(x_traj, z_traj, style_dict[i], label=trajectories[i])

        # map trajectory
        # x_map = [pose[0] for pose in map_traj]
        # z_map = [pose[2] for pose in map_traj]
        # plt.plot(x_map, z_map, style_map, label='map_traj')

        # set the range of x and y
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        xmean = np.mean(xlim)
        ymean = np.mean(ylim)
        plot_radius = max([abs(lim - mean_)
                           for lims, mean_ in ((xlim, xmean),
                                               (ylim, ymean))
                           for lim in lims]) + 3
        ax.set_xlim([xmean - plot_radius, xmean + plot_radius])
        ax.set_ylim([ymean - plot_radius, ymean + plot_radius])

        #  plot the map as a scatter plot of points
        x_map = [point[0] for point in map_points]
        z_map = [point[2] for point in map_points]

        # gets each points color according to its y value on the map
        # map_gradiant = self.get_map_gradiant(culled_map)
        # point_color = self.get_point_colors(map_points)

        # plot the map as a scatter plot
        # plt.scatter(x_map, z_map, s=0.4, c=point_color, cmap="RdYlBu")
        plt.scatter(x_map, z_map, s=0.4, c="black")

        # plot start point
        plt.plot(start_point[0], start_point[1], style_start_point, label='Start Point')

        # Add separate legend item for coverage so it doesn't duplicate
        handles, labels = ax.get_legend_handles_labels()
        coverage_legend_item = mpatches.Patch(color='gold', label='Coverage')
        handles.append(coverage_legend_item)

        plt.legend(loc="upper right", prop={'size': font_size}, handles=handles)
        png_title = "Trajectories"
        plt.savefig(self.result_dir + "/" + png_title + ".png", bbox_inches='tight', pad_inches=0.1)
        plt.savefig(self.result_dir + "/" + png_title + ".pdf", bbox_inches='tight', pad_inches=0.1)

        plt.show()
        # plt.close()

    def generate_report(self, toCameraCoord):
        """
            to_camera_coord: whether the predicted pose needs to be convert to camera coordinate space
        """

        map_points = fl.load_map(self.map_file)
        map_trajectory = fl.load_poses(self.map_trajectory)

        '''
            Rotates both the map trajectory and the maps point cloud so that the trajectory is in the horizontal plane
            First rotates around the horizontal axis with the biggest deviation in the angle, then the other 
            horizontal axis
            Assumes that the surface is a plain, wont work for trajectories with changes in elevation
        '''
        map_trajectory, map_points = tf.rotate_map_and_trajectory(map_trajectory, map_points)

        #  removes points above and below a certain threshold on the vertical axis
        map_points = fl.cull_map(map_points, self.ceiling, self.floor)

        self.plot_trajectories_2d(self.trajectories, map_points, map_trajectory, toCameraCoord=toCameraCoord)
        #self.plot_trajectories_3d(self.trajectories, map_points, map_trajectory, toCameraCoord=toCameraCoord)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SLAM report generator')
    parser.add_argument('--traj_dir', type=str,
                        help='Path to directory that contains the trajectories that are to be evaluated')
    parser.add_argument('--result_dir', type=str,
                        help='Path to directory where the results will be stored')
    parser.add_argument('--map_dir', type=str,
                        help='Path to directory that contains the map.msg file and it\'s corresponding trajectory')
    parser.add_argument('--ceiling', type=float, default=-999,
                        help='Threshold of the maps ceiling. All points above this threshold will be removed from the drawn map')
    parser.add_argument('--floor', type=float, default=-999,
                        help='Threshold below which all the map points will be removed from the drawn map.')
    parser.add_argument('--toCameraCoord', type=lambda x: (str(x).lower() == 'true'), default=False,
                        help='Whether to convert the pose to camera coordinate')
    parser.add_argument('--brush_width', type=float, default=-999,
                        help='The width of the brushes')
    parser.add_argument('--config', type=str, default='',
                        help='Config file path (NOTE: flags take priority over config)')

    args = parser.parse_args()
    report_gen = ReportGenerator(args)
    report_gen.generate_report(toCameraCoord=args.toCameraCoord)
