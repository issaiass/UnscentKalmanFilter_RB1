#! /usr/bin/env python
import rospy
import rospkg
import yaml
from rb1_navigation.srv import msg, msgResponse
from geometry_msgs.msg import PoseStamped


class Server:
    def __init__(self):
        rospy.init_node("navigate")
        # =========== #
        self.get_path()
        self.loadParameters()
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",
                                        PoseStamped, queue_size=1)

    def create(self):
        rospy.loginfo("[INFO] Creating service '/go_to_point'")
        self.srv = rospy.Service("/go_to_point", msg, self.my_callback)
        rospy.loginfo("[INFO] Service Created '/go_to_point'")

    def my_callback(self, req):
        """ msg 
        string label
        ---
        bool val
        """
        goal = self.Goal(req.label)
        resp = msgResponse()

        if goal is None:
            rospy.logerr("[ERROR] Location is not in databse. Avalailable locations are:{}.".format(
                self.storedContent.keys()))
            resp.val = False
        else:
            print(goal)
            self.goal_pub.publish(goal)
            print("[INFO] Waypoint sent")
            resp.val = True
        return resp

    # -  load way points from rb1_localization
    def Goal(self, loc):
        if loc in self.storedContent.keys():
            return self.createGoal(self.storedContent[loc])
        else:
            return None

    def createGoal(self, info):
        targ = PoseStamped()
        targ.header.frame_id = "map"
        targ.header.stamp = rospy.Time.now()
        targ.pose.position.x = info["position"]["x"]
        targ.pose.position.y = info["position"]["y"]
        targ.pose.position.z = info["position"]["z"]
        targ.pose.orientation.x = info["orientation"]["x"]
        targ.pose.orientation.y = info["orientation"]["y"]
        targ.pose.orientation.z = info["orientation"]["z"]
        targ.pose.orientation.w = info["orientation"]["w"]

        return targ

    def get_path(self, localization_package_name="rb1_localization"):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path(localization_package_name)

    def loadParameters(self, filename="poi.yaml"):
        fullPath = self.path + "/params/" + filename

        with open(fullPath, "r") as stream:
            try:
                content = yaml.safe_load(stream)

            except yaml.YAMLError as exc:
                print(exc)

        self.storedContent = content


if __name__ == "__main__":
    a = Server()
    a.loadParameters()
    a.create()

    rospy.spin()
