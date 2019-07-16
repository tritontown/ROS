import apriltag
import cv2
import numpy as np
import time
import math
import rospy
from geometry_msgs.msg import PoseStamped
from tag_sim.msg import tagInfoArray
from tag_sim.msg import tagInfo
from tf.transformations import quaternion_from_euler as makeQuat

'''
#draw dots on tag corners
def draw_tag(frame,result):
        angle = []
        for i in range(len(result)):
                cv2.circle(frame,tuple(result[i].center.astype(int)),5,(0,0,255),3)
                cv2.circle(frame,tuple(result[i].corners[0].astype(int)),5,(0,0,255),3)
                cv2.circle(frame,tuple(result[i].corners[1].astype(int)),5,(0,255,0),3)
                cv2.circle(frame,tuple(result[i].corners[2].astype(int)),5,(255,0,0),3)
                cv2.circle(frame,tuple(result[i].corners[3].astype(int)),5,(0,0,0),3)
                mid = [np.int((result[i].corners[0][0]+result[i].corners[1][0])/2),np.int((result[i].corners[0][1]+result[i].corners[1][1])/2)]
                cv2.line(frame,tuple(result[i].center.astype(int)),tuple(mid),(0,0,255),5)
                angle.append(math.degrees(math.atan2(result[i].corners[1][1]-result[i].corners[0][1],result[i].corners[1][0]-result[i].corners[0][0])))
        return frame, angle 
'''

#draw box on tags
def draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):
    opoints = np.array([
        -1, -1, 0,
        1, -1, 0,
        1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
        1, -1, -2*z_sign,
        1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
        ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
        ]).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3,dtype=R.dtype)
    n = np.linalg.norm(I-shouldBeIdentity)
    return n<1e-6

def tagDataToPose(pose, tagId):

    angles = rotationMatrixToEulerAngles(pose[0:3,0:3])
    center = pose[0:3, 3]

    # Convert angles to Quaternion space
    quat = makeQuat(angles[0], angles[1], angles[2])

    # Init Pose Obj
    tagPose = PoseStamped()

    # Pose creation time saved, may be useful later
    #tagPose.header = rospy.get_rostime()

    # Position
    tagPose.pose.position.x = center[0]
    tagPose.pose.position.y = center[1]
    tagPose.pose.position.z = center[2]

    # Orientation
    tagPose.pose.orientation.x = quat[0]
    tagPose.pose.orientation.y = quat[1]
    tagPose.pose.orientation.z = quat[2]
    tagPose.pose.orientation.w = quat[3]

    # This is fairly inefficient, but makes code easier to read
    tag = tagInfo()
    tag.pose = tagPose
    tag.id = tagId

    return tag

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0]*R[0,0]+R[1,0]*R[1,0])
    singular = sy<1e-6
    if not singular:
        x = math.atan2(R[2,1],R[2,2])
        y = math.atan2(-R[2,0],sy)
        z = math.atan2(R[1,0],R[0,0])
    else:
        x = math.atan2(-R[1,2],R[1,1]) 
        y = math.atan2(-R[2,0],sy)
        z = 0
    return np.array([x,y,z])

def main():
   
    DIM = (640, 480)
    camera_params = (506.66588415210174,507.57637424966526,311.03765199523536,238.60300515336095)
    tag_size = 160
    video_capture = cv2.VideoCapture(0)
    detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
   
    # ROS Publisher Init
    pub = rospy.Publisher('tagPose', tagInfoArray, queue_size=5)
    rospy.init_node('talker', anonymous=True)

    # Init array of tag poses
    #tagPoses = tagInfoArray()

    while (True):
        
        _, frame = video_capture.read()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = detector.detect(gray_frame)

        # Init fresh tag arr to pass
        tagPoses = tagInfoArray()

        '''
        if (result):
            frame, angle = draw_tag(frame,result)
            print(angle)
        '''


        for i, detection in enumerate(result):
            pose, e0, e1 = detector.detection_pose(detection,camera_params,tag_size)
            draw_pose(frame, camera_params,tag_size,pose)

            #print(result[i].tag_id)
            tagPoses.tags.append( tagDataToPose(pose, result[i].tag_id) )

           # print(angles,pose[0:3,3])	

        print(tagPoses)

        pub.publish(tagPoses)

        cv2.imshow("apriltags",frame)
        k = cv2.waitKey(1)
  
        if( k % 256 == 32):
            break

        """
        except KeyboardInterrupt:
            break
        """
    video_capture.release()

if __name__=="__main__":
    
    try:
        main()

    except rospy.ROSInterruptException:
        pass
