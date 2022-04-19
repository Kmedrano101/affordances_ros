#!/usr/bin/env python3

# Importar modulos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from affordances.msg import personPos,pose 
import mediapipe as mp
import time
from os import system
import cv2 as cv
from copy import deepcopy

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_pp_name", default="person_pose")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/person_object_flag/topic", default="/affordances/flag")
TOPIC_S2_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/image_source/topic", default="/usb_cam/image_raw")
TOPIC_P1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_person_pose/topic", default="/affordances/status_person_pose")
TOPIC_P2_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_person_pose/topic", default="/affordances/data_person_pose")
BRIDGE = CvBridge()
# Clase Person Pose


class PersonPose:
    """Person object"""

    def __init__(self):
        self._cvFrame = []						# Only Private no external access
        self._subTopicFlagName = None           #
        self._subTopicImageSourceName = None    # Private and external access
        self._pubTopicStatusNodeName = None		# Private and external access
        self._pubTopicDataNodeName = None	    # Private and external access
        self._pubTopicDataNode = None	        # Private and external access
        self._pubTopicStatusNode = None			# Private and external access
        self._statusFlag = False				# Private and external access
        self._dataReceivedTopic1 = False		# Private and external access
        self._dataReceivedTopic2 = False		# Private and external access
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose()
        self._readyCapturePose = False	        # Private and external access

    """Properties"""
    @property
    def readyCapturePose(self):
        """The readyCapturePose property."""
        return self._readyCapturePose

    @readyCapturePose.setter
    def readyCapturePose(self, value):
        self._readyCapturePose = value

    @property
    def cvFrame(self):
        """The cvFrame property."""
        return self._cvFrame

    @cvFrame.setter
    def cvFrame(self, value):
        self._cvFrame = value

    @property
    def subTopicFlagName(self):
        """The subTopicFlagName property."""
        return self._subTopicFlagName

    @subTopicFlagName.setter
    def subTopicFlagName(self, value):
        self._subTopicFlagName = value  

    @property
    def subTopicImageSourceName(self):
        """The subTopicImageSourceName property."""
        return self._subTopicImageSourceName

    @subTopicImageSourceName.setter
    def subTopicImageSourceName(self, value):
        self._subTopicImageSourceName = value

    @property
    def pubTopicStatusNodeName(self):
        """The pubTopicStatusNodeName property."""
        return self._pubTopicStatusNodeName
    @pubTopicStatusNodeName.setter

    def pubTopicStatusNodeName(self, value):
        self._pubTopicStatusNodeName = value        

    @property
    def pubTopicDataNodeName(self):
        """The pubTopicDataNodeName property."""
        return self._pubTopicDataNodeName

    @pubTopicDataNodeName.setter
    def pubTopicDataNodeName(self, value):
        self._pubTopicDataNodeName = value

    @property
    def pubTopicDataNode(self):
        """The pubTopicdataNode property."""
        return self._pubTopicDataNode

    @pubTopicDataNode.setter
    def pubTopicDataNode(self, value):
        self._pubTopicDataNode = value

    @property
    def dataReceivedTopic1(self):
        """The dataReceivedTopic1 property."""
        return self._dataReceivedTopic1

    @dataReceivedTopic1.setter
    def dataReceivedTopic1(self, value):
        self._dataReceivedTopic1 = value        

    @property
    def dataReceivedTopic2(self):
        """The dataReceivedTopic2 property."""
        return self._dataReceivedTopic2

    @dataReceivedTopic2.setter
    def dataReceivedTopic2(self, value):
        self._dataReceivedTopic2 = value

    """ Methods and Actions"""
    def find_pose(self, draw=True) -> None:
        imgRGB = cv.cvtColor(self.cvFrame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    self.cvFrame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)

    def find_position(self, draw=True) -> None:
        lmAux = personPos()
        dataPoint = Point()
        dataPose = pose()
        contador = 0
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                #print(f"id {id} x: {lm.x} y: {lm.y}")
                dataPoint.x = lm.x
                dataPoint.y = lm.y
                dataPoint.z = lm.z
                dataPose.id_pos = id
                dataPose.position = dataPoint
                dataPose.visibility = lm.visibility
                if id == 0 or id == 15 or id == 16 or id == 11 or id == 12:
                    if lm.visibility>0.5:
                        contador += 1
                h, w, c = self.cvFrame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmAux.poses.append(deepcopy(dataPose))
                #if draw:
                #    cv.circle(self.cvFrame, (cx, cy), 5, (255, 0, 0), cv.FILLED)
            if contador >= 3:
                self.readyCapturePose = True
            else:
                self.readyCapturePose = False
        self.pubTopicDataNode.publish(lmAux)

    def image_source_callback(self, msg) -> None:
        self._dataReceivedTopic1 = True
        try:
            self._cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ERROR] {e}")

    def flag_callback(self,msg) -> None:
        self._dataReceivedTopic2 = True
        self._statusFlag = msg

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicImageSourceName, Image, self.image_source_callback)
        rospy.Subscriber(self._subTopicFlagName,Bool, self.flag_callback)

    def start_publishers(self) -> None:
        self._pubTopicStatusNode = rospy.Publisher(self._pubTopicStatusNodeName, Bool, queue_size=10)
        self._pubTopicDataNode = rospy.Publisher(self._pubTopicDataNodeName,personPos, queue_size=10)


def main():
    # Don't forget to remove this test mode
    system('clear')
    time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    #rate = rospy.Rate(1.0)
    """Inicializar el objeto object_pos"""
    objNode = PersonPose()
    objNode.subTopicFlagName = TOPIC_S1_NAME
    objNode.subTopicImageSourceName = TOPIC_S2_NAME
    objNode.pubTopicStatusNodeName = TOPIC_P1_NAME
    objNode.pubTopicDataNodeName = TOPIC_P2_NAME
    objNode.start_subscribers()
    objNode.start_publishers()
    time.sleep(2)
    INFO1 = True
    INFO2 = True
    while not rospy.is_shutdown():
        # or objNode._dataReceivedTopic2 == False:
        if not objNode.dataReceivedTopic1  and not objNode.dataReceivedTopic2 and INFO2:
            print("[WARNING] Datos no recibidos")
            INFO2 = False
        else:
            #time.sleep(1.5)
            objNode.find_pose()
            objNode.find_position(draw=False)
            cv.imshow("Image from Node PersonPose", objNode.cvFrame)
            cv.waitKey(1)
            objNode._pubTopicStatusNode.publish(objNode.readyCapturePose)
            if objNode.readyCapturePose and INFO1:
                rospy.loginfo("READY TO CAPTURE DATA")
                INFO1 = False
            #else:
            #    rospy.loginfo("NOT READY TO CAPTURE DATA")
            #    INFO1 = True
        #rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
