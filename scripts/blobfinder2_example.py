#!/usr/bin/env python

import roslib; roslib.load_manifest('project1')
import rospy
import random

from blobfinder2.msg import MultiBlobInfo

def on_blob_msg(msg):

    max_area = None
    max_cx = None

    for cblob in msg.color_blobs:
        if cblob.color.data == 'red_tape':
            for blob in cblob.blobs:
                if max_area is None or blob.area > max_area:
                    max_area = blob.area
                    max_cx = blob.cx

    if max_area is None:
        rospy.loginfo('no red_tape blobs found!')
    else:
        rospy.loginfo('biggest red_tape blob had area %f at x=%f',
                      max_area, max_cx)

def main():

    rospy.init_node('blobfinder2_example', anonymous=True)
    
    rospy.Subscriber('blobfinder2/blobs', MultiBlobInfo, on_blob_msg)

    rospy.spin()

# main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
