#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger

class AlignServiceClient:
    def __init__(self):
        # rospy.wait_for_service('/start_align')
        self.proxy = rospy.ServiceProxy('/start_align', Trigger)

    def call(self):
        try:
            resp = self.proxy()
            return resp.success, resp.message
        except rospy.ServiceException as e:
            return False, str(e)
   

