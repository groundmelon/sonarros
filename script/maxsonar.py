#!/usr/bin/env python

import serial
import rospy
import select
from geometry_msgs.msg import Vector3Stamped

if __name__ == "__main__":
    rospy.init_node('maxsonar')

    devname = rospy.get_param("~devname", "/dev/ttyUSB0")
    bdrate = rospy.get_param("~baudrate", 9600)
    limit_low = rospy.get_param("~limit_low", 0.21)
    limit_high = rospy.get_param("~limit_high", 4.0)
    install_height = rospy.get_param("~install_height", 0.13)

    pub = rospy.Publisher("~height", Vector3Stamped, queue_size=10)
    pub_raw = rospy.Publisher("~height_raw", Vector3Stamped, queue_size=10)

    try:
        ser = serial.Serial(port=devname, baudrate=bdrate, timeout=10)
        rospy.loginfo("Port <%s> open successfully." % devname)
    except Exception, e:
        raise e

    try:
        while not rospy.is_shutdown():
            buf = ser.read(1)
            if buf[0] == 'R':
                ser.read(4)
                break

        while not rospy.is_shutdown():
            buf = ser.read(5)
            if buf[0] == 'R':
                height = float(int(buf[1:4])) / 100.0
                msg = Vector3Stamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "sonar"
                msg.vector.z = height
                pub_raw.publish(msg)

                if height < limit_low:
                    # too near
                    msg.vector.z = install_height
                    pub.publish(msg)
                elif height > limit_high:
                    # too far
                    pass
                else:
                    # normal
                    pub.publish(msg)
            else:
                while not rospy.is_shutdown():
                    buf = ser.read(1)
                    if buf[0] == '\n':
                        break
    except serial.SerialTimeoutException, e:
        rospy.logerr("MaxSonar read timeout!")
    except select.error:
        rospy.logerr("Ctrl-C singal, exit...")
    finally:
        ser.close()
