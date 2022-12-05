#! /usr/bin/env python
from collections import namedtuple
import rospkg


class Storage:
    loc_tuple = namedtuple("Loc", ["name", "p_x", "p_y", "p_z",
                                   "q_x", "q_y", "q_z", "q_w"])

    def __init__(self):
        """Handles storage information"""
        self.curr_locations = []
        self.get_path()

    def get_path(self):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path("rb1_localization")

    def add_loc(self, name, odom):
        """- Parameters required
        geometry_msgs/PoseWithCovariance pose
            geometry_msgs/Pose pose
                geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
                geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        """
        self.curr_locations.append(self.loc_tuple(name,
                                                  odom.pose.pose.position.x,
                                                  odom.pose.pose.position.y,
                                                  odom.pose.pose.position.z,
                                                  odom.pose.pose.orientation.x,
                                                  odom.pose.pose.orientation.y,
                                                  odom.pose.pose.orientation.z,
                                                  odom.pose.pose.orientation.w))

    def file_format(self, msg):
        s_msg = "{}:\n".format(msg.name) +\
                " position:\n" +\
                "  x: {}\n".format(msg.p_x) + \
                "  y: {}\n".format(msg.p_y) + \
                "  z: {}\n".format(msg.p_z) + \
                " orientation:\n" +\
                "  x: {}\n".format(msg.q_x) + \
                "  y: {}\n".format(msg.q_y) + \
                "  z: {}\n".format(msg.q_z) + \
                "  w: {}\n".format(msg.q_w)

        return s_msg

    def store(self, file_name):
        path = self.path + "/params/{}.yaml".format(file_name)

        with open(path, "w") as f:
            for pos in self.curr_locations:
                f.write(self.file_format(pos))
                f.write("\n")
