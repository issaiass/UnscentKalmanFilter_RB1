#! /usr/bin/env python
from rb1_localization.srv import LocMsg, LocMsgResponse
from nav_msgs.msg import Odometry
from loc_storage import Storage
import rospy


class LocationStorage:
    def __init__(self):
        self._check_odom_ready()
        self.odom_msg = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.poi_srv = rospy.Service("/save_poi", LocMsg, self.poi_callback)
        self.storage = Storage()
        self.resp = LocMsgResponse()

    def poi_callback(self, req):
        """
            string label
            ---
            bool success
        """
        loc_name = req.label

        if "position" in loc_name:
            rospy.loginfo("[INFO] Storing: {} in database".format(loc_name))
            self.storage.add_loc(loc_name, self.odom_msg)
            self.resp.success = "[INFO] {} stored succesfully".format(loc_name)

        elif "end" in loc_name:
            self.storage.store("poi")
            self.resp.success = "[INFO] Locations stored in params folder"

        else:
            rospy.logerr("[WARN] Wrong msg passed...")

        return self.resp

    def odom_callback(self, msg):
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
        self.odom_msg = msg

    def _check_odom_ready(self):
        self.odom_msg = None
        rospy.loginfo("[INFO] Checking if odom topic is ready...")
        while self.odom_msg is None and not rospy.is_shutdown():
            try:
                self.odom_msg = rospy.wait_for_message(
                    "/odom", Odometry, timeout=1)
            except:
                rospy.logerr("[CAUTION] Waiting for odom topic...")
        rospy.loginfo("[INFO] Odometry topic connection succesful...")

        return self.odom_msg


if __name__ == "__main__":
    rospy.init_node("poi_server")
    LocationStorage()
    rospy.spin()
