#!/usr/bin/env python

import rospy
#from cmvision.msg import Blobs, Blob
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from tag_sim.msg import tagInfo
from tag_sim.msg import tagInfoArray

"""
class MarkerBasics(object):
    #__slots__ = ['tagData']

    #marker_objectlisher = rospy.Publisher('/tagLoc', Marker, queue_size=10)

    def __init__(self, tagData):
        #self.marker_objectlisher = rospy.Publisher('/tagLoc', Marker, queue_size=10)
        #self.rate = rospy.Rate(1)
        #self.tagData = tagData
        self.init_marker(tagData)

    def init_marker(self, tagData):
        print("double fuck")
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/base_link"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "tag"
        self.marker_object.id = tagData.id
        self.marker_object.type = Marker.CUBE
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.position.x = tagData.pose.pose.position.x
        self.marker_object.pose.position.y = tagData.pose.pose.position.y
        self.marker_object.pose.position.z = tagData.pose.pose.position.z - 500

        self.marker_object.pose.orientation.x = tagData.pose.pose.orientation.x
        self.marker_object.pose.orientation.y = tagData.pose.pose.orientation.y
        self.marker_object.pose.orientation.z = tagData.pose.pose.orientation.z
        self.marker_object.pose.orientation.w = tagData.pose.pose.orientation.w
        
        self.marker_object.scale.x = 5 * tagData.pose.pose.position.z / 100
        self.marker_object.scale.y = 5 * tagData.pose.pose.position.z / 100
        self.marker_object.scale.z = 5 * tagData.pose.pose.position.z / 100

        self.marker_object.color.r = 1.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 0.0

        # This has to be otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)


        #print( 'In initMarker' )

        #self.marker_objectlisher.publish(self.marker_object)

    def update_position(self, tagData):       
        self.marker_object.pose.position.x = tagData.pose.pose.position.x
        self.marker_object.pose.position.y = tagData.pose.pose.position.y
        self.marker_object.pose.position.z = tagData.pose.pose.position.z - 500

        self.marker_object.pose.orientation.x = tagData.pose.pose.orientation.x
        self.marker_object.pose.orientation.y = tagData.pose.pose.orientation.y
        self.marker_object.pose.orientation.z = tagData.pose.pose.orientation.z
        self.marker_object.pose.orientation.w = tagData.pose.pose.orientation.w
        
        self.marker_object.scale.x = 5 * tagData.pose.pose.position.z / 100
        self.marker_object.scale.y = 5 * tagData.pose.pose.position.z / 100
        self.marker_object.scale.z = 5 * tagData.pose.pose.position.z / 100


        self.marker_object.header.stamp = rospy.get_rostime()

        #self.marker_objectlisher.publish(self.marker_object)
"""
class TagDetector(object):
   

    def __init__(self):        
        self.rate = rospy.Rate(1)

        self.markArr = MarkerArray()
        self.markArrPub = rospy.Publisher('/tagLoc', MarkerArray, queue_size=10)
        
        #self.markArr = MarkerArray()
        print("init tagdet")
        rospy.Subscriber('tagPose', tagInfoArray, self.tag_callback)        
        
   
    def tag_callback(self,data):
    
        for tag in data.tags:
            
            found = False
            
            for marker in self.markArr.markers:
                print(tag.id,marker.id)
                if(tag.id == marker.id):
                    self.update_position( marker, tag )
                    found = True
                    break

            if(not found):
                marker = Marker()
                self.init_marker(marker, tag)
                self.markArr.markers.append(marker)
                print('fuck')



        
        self.markArrPub.publish(self.markArr.markers)


    def start_loop(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def init_marker(self, marker, tagData):
        print("double fuck")
        
        marker.header.frame_id = "/base_link"
        marker.header.stamp    = rospy.get_rostime()
        marker.ns = "tag"
        marker.id = tagData.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = tagData.pose.pose.position.x
        marker.pose.position.y = tagData.pose.pose.position.y
        marker.pose.position.z = -1 * tagData.pose.pose.position.z / 50

        marker.pose.orientation.x = tagData.pose.pose.orientation.x
        marker.pose.orientation.y = tagData.pose.pose.orientation.y
        marker.pose.orientation.z = tagData.pose.pose.orientation.z
        marker.pose.orientation.w = tagData.pose.pose.orientation.w
        
        marker.scale.x = 5 * tagData.pose.pose.position.z / 100
        marker.scale.y = 5 * tagData.pose.pose.position.z / 100
        marker.scale.z = 5 * tagData.pose.pose.position.z / 100

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # This has to be otherwise it will be transparent
        marker.color.a = 1.0

        # If we want it for ever, 0, otherwise seconds before desapearing
        marker.lifetime = rospy.Duration(0)

        #print( 'In initMarker' )

        #self.marker_objectlisher.publish(self.marker_object)



    def update_position(self, marker, tagData):       
        marker.pose.position.x = tagData.pose.pose.position.x
        marker.pose.position.y = tagData.pose.pose.position.y
        marker.pose.position.z = tagData.pose.pose.position.z

        marker.pose.orientation.x = tagData.pose.pose.orientation.x
        marker.pose.orientation.y = tagData.pose.pose.orientation.y
        marker.pose.orientation.z = tagData.pose.pose.orientation.z
        marker.pose.orientation.w = tagData.pose.pose.orientation.w
        
        marker.scale.x = 5 * tagData.pose.pose.position.z / 100
        marker.scale.y = 5 * tagData.pose.pose.position.z / 100
        marker.scale.z = 5 * tagData.pose.pose.position.z / 100


        marker.header.stamp = rospy.get_rostime()

        #self.marker_objectlisher.publish(self.marker_object)



if __name__ == '__main__':
    rospy.init_node('tag_detections_listener_node', anonymous=True)
    print(1)
    tag_detector_object = TagDetector()
    print(2)
    tag_detector_object.start_loop()
    print(3)
