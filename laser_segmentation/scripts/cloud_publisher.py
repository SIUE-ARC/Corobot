#!/usr/bin/env python
import roslib
import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

rospy.init_node("cloud_publisher")
rospy.wait_for_service("/assemble_scans")

r = rospy.Rate(10)

assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

pub = rospy.Publisher('cloud', PointCloud2, queue_size=10)

last_time = rospy.Time.now()
while not rospy.is_shutdown():
    try:
        now = rospy.Time.now()

        resp = assemble_scans(last_time, now)
        last_time = now

        if(resp.cloud.data):
            pub.publish(resp.cloud)
            rospy.logdebug("Got cloud with %u points" % resp.cloud.width)

        else:
            rospy.logwarn("Got empty cloud")

    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)

    try:
        r.sleep()
    except rospy.exceptions.ROSInterruptException as e:
        rospy.logdebug("Sleep interrupted: %s" % e)

