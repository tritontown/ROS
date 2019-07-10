#!/usr/bin/env python
import rospy
#from cmvision.msg import Blobs, Blob
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
#from sensor_msgs.msg import CameraInfo


class MarkerBasics(object):

    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/marker_redball', Marker, queue_size=10)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)

    def init_marker(self,index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/base_link"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "mira"
        self.marker_object.id = index
        self.marker_object.type = Marker.CUBE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 1
        self.marker_object.pose.orientation.y = 1
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 10
        self.marker_object.scale.y = 10
        self.marker_object.scale.z = 0.000001

        self.marker_object.color.r = 1.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 0.0

        # This has to be otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)

    def update_position(self,position,angle):       
        self.marker_object.pose.position.x = position.x - 320
        self.marker_object.pose.position.y = position.y - 240

        q = quaternion_from_euler(0,0,angle)
        self.marker_object.pose.orientation.x = q[0]
        self.marker_object.pose.orientation.y = q[1]
        self.marker_object.pose.orientation.z = q[2]
        self.marker_object.pose.orientation.w = q[3]

        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_objectlisher.publish(self.marker_object)
        


class BallDetector(object):
    def __init__(self):        
        self.rate = rospy.Rate(1)
        #self.save_camera_values()        
        rospy.Subscriber('tagPose', Pose, self.redball_detect_callback)        
        self.markerbasics_object = MarkerBasics()

    """
    def save_camera_values(self):
        data_camera_info = None
        while data_camera_info is None:
            data_camera_info = rospy.wait_for_message('/mira/mira/camera1/camera_info', CameraInfo, timeout=5)
            rospy.loginfo("No Camera info found, trying again")
    
    
        self.cam_height_y = data_camera_info.height
        self.cam_width_x = data_camera_info.width
        rospy.loginfo("CAMERA INFO:: Image width=="+str(self.cam_width_x)+", Image Height=="+str(self.cam_height_y))
    """
    
    def redball_detect_callback(self,data):

        """
        if(len(data.blobs)):

            for obj in data.blobs:
                if obj.name == "RedBall":
                    rospy.loginfo("Blob <"+str(obj.name)+"> Detected!")
                    redball_point = Point()
                    # There is a diference in the axis from blobs and the camera link frame.
                    # We convert to percent of the screen
                    # TODO: Take into account the Depth distance and camera cone.
                    rospy.loginfo("self.cam_width_x="+str(self.cam_width_x))
                    rospy.loginfo("self.cam_width_x="+str(self.cam_height_y))
                    rospy.loginfo("obj.x="+str(obj.x))
                    rospy.loginfo("obj.y="+str(obj.y))

                    middle_width = float(self.cam_width_x)/2.0 
                    middle_height = float(self.cam_height_y)/2.0

                    redball_point.x = (obj.x - middle_width) / float(self.cam_width_x)
                    redball_point.z = (obj.y - middle_height) / float(self.cam_height_y)                    
                    redball_point.y = 0.6            
                    rospy.loginfo("blob is at Point="+str(redball_point))
                    self.markerbasics_object.update_position(position=redball_point)


        else: 
             rospy.logwarn("No Blobs Found")
    """

        #redball_point = Point()
        #redball_point.x = data.point.x
        #redball_point.y = data.point.y
        self.markerbasics_object.update_position(data.position, angle=data.orientation.z)

    def start_loop(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('redball_detections_listener_node', anonymous=True)
    redball_detector_object = BallDetector()
    redball_detector_object.start_loop()
