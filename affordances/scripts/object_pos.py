#!/usr/bin/env python3

# Importar modulos
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
from affordances.msg import objectData
import time
from os import system
import cv2 as cv

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_op_name", default="object_pos")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/border_boxes/topic", default="/affordances/object_bounding_boxes")
TOPIC_S2_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/image_source/topic", default="/usb_cam/image_raw")
TOPIC_P1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_object_pos/topic", default="/affordances/status_object_pos")
TOPIC_P2_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_object_pos/topic", default="/affordances/data_object_pos")

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv.TrackerCSRT_create,
    "kcf": cv.TrackerKCF_create,
    "mil": cv.TrackerMIL_create
}

BRIDGE = CvBridge()

# Class ObjectPos


class ObjectPos:
    """Person object"""

    def __init__(self):
        self._subTopicBorderBoxesName = None
        self._subTopicImageName = None
        self._pubTopicStatusNodeName = None
        self._pubTopicDataNodeName = None
        self.pubTopicStatusNode = None
        self.pubTopicDataNode = None
        self.objectPos = None
        self.cvFrame = []
        self.statusNode = False
        self.borderBoxes = ()
        self.dataReceivedTopic1 = False
        self.dataReceivedTopic2 = False
        self.dataPosReady = False
        self.posObject = ()
        self.sTime = 0.1
        self.iTime = 0
        self.fValue = 0
        self.TrackTime = 9.5
        self.TrackUpdateTime = 0
        self.X1 = 0
        self.Y1 = 0
        self.ListVelocities = []
        self.velocityObject = 0
        self.data = objectData()
        self.statusTracking = False
        self.resetTracker = False
        self.auxReset = False
        self.TRACKER = OPENCV_OBJECT_TRACKERS["csrt"]()
        self.ClassObject = None

    """Propierties"""
    @property
    def subTopicBorderBoxesName(self):
        """The subTopicBorderBoxesName property."""
        return self._subTopicBorderBoxesName

    @subTopicBorderBoxesName.setter
    def subTopicBorderBoxesName(self, value):
        if value:
            self._subTopicBorderBoxesName = value
        else:
            rospy.loginfo("Invalid Name of topic BorderBoxes")

    @property
    def subTopicImageName(self):
        """The subTopicImageName property."""
        return self._subTopicImageName

    @subTopicImageName.setter
    def subTopicImageName(self, value):
        if value:
            self._subTopicImageName = value
        else:
            rospy.loginfo("Invalid Name of topic Image")

    @property
    def pubTopicStatusNodeName(self):
        """The pubTopicStatusNodeName property."""
        return self._pubTopicStatusNodeName

    @pubTopicStatusNodeName.setter
    def pubTopicStatusNodeName(self, value):
        if value:
            self._pubTopicStatusNodeName = value
        else:
            rospy.loginfo("Invalid Name of topic Status")

    @property
    def pubTopicDataNodeName(self):
        """The pubTopicDataNodeName property."""
        return self._pubTopicDataNodeName

    @pubTopicDataNodeName.setter
    def pubTopicDataNodeName(self, value):
        if value:
            self._pubTopicDataNodeName = value
        else:
            rospy.loginfo("Invalid Name of topic Data")

    """ Methods and Actions"""

    def image_source_callback(self, msg) -> None:
        self.dataReceivedTopic1 = True
        try:
            self.cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
            #self._cvFrame = cv.resize(self._cvFrame, (416, 416))
        except CvBridgeError as e:
            print(f"[ERROR] {e}")

    def border_boxes_callback(self, msg) -> None:
        self.dataReceivedTopic2 = True
        self.borderBoxes = msg.bounding_boxes

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicImageName, Image,
                        self.image_source_callback)
        rospy.Subscriber(self._subTopicBorderBoxesName,
                        BoundingBoxes, self.border_boxes_callback)

    def start_publishers(self) -> None:
        self.pubTopicStatusNode = rospy.Publisher(
            self._pubTopicStatusNodeName, Bool, queue_size=10)
        self.pubTopicDataNode = rospy.Publisher(
            self._pubTopicDataNodeName, objectData, queue_size=10)

    def get_pos(self) -> None:
        """ Funcion para obtener la posicion central del objeto a realizar el tracking"""
        pos = []
        self.dataPosReady = False
        # ["refrigerator","cup","bicycle","sofa","chair","wine glass","cell phone","laptop","book"]
        Objetos = ["tvmonitor"]
        if self.dataReceivedTopic2 and not self.statusTracking:
            for box in self.borderBoxes:
                if box.Class in Objetos:
                    pos.append(box.xmin-2)
                    pos.append(box.ymin-2)
                    pos.append(abs(box.ymax - box.ymin)+2)
                    pos.append(abs(box.xmax - box.xmin)+2)
                    self.dataPosReady = True
                    rospy.loginfo("Object detected")
                    self.posObject = tuple(pos)
                    self.borderBoxes = []
                    self.ClassObject = box.Class
                    break

    def find_velocity(self, x, y) -> None:
        if self.fValue == 0:
            self.X1, self.Y1 = x, y
            self.fValue = 1
        self.aTime = time.time()
        if self.aTime > (self.sTime+self.iTime):
            X2, Y2 = x, y
            V = abs(self.X1 - X2) + abs(self.Y1 - Y2)
            self.ListVelocities.append(V)
            if len(self.ListVelocities) >= 5:
                # FPS In this case is just per 1s
                self.velocityObject = (
                    sum(self.ListVelocities)/len(self.ListVelocities)) * 2
                self.ListVelocities = []
            self.fValue = 0
            self.iTime = time.time()

    def process_img(self) -> None:
        if self.resetTracker and not self.auxReset:
            self.restart_Tracker()
            self.auxReset = True
        if self.dataPosReady and self.dataReceivedTopic1 and not self.statusNode:
            self.statusTracking = True
            if self.posObject:
                self.TRACKER.init(self.cvFrame, self.posObject)
                rospy.loginfo("Tracker started.")
                self.auxReset = False
            self.statusNode = True
        if self.statusTracking:
            try:
                (success, box) = self.TRACKER.update(self.cvFrame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv.rectangle(self.cvFrame, (x, y),
                                (x + h, y + w), (0, 255, 0), 2)
                    X = int((x*2+h)/2)
                    Y = int((y*2+w)/2)
                    self.find_velocity(X, Y)
                    self.data.x = X
                    self.data.y = Y
                    self.data.velocity = self.velocityObject
                    self.data.classObject = self.ClassObject
                    cv.circle(self.cvFrame, (X, Y), 4, (255, 0, 0), -1)
                else:
                    self.resetTracker = True
            except Exception as e:
                print("[Error] updating IMG", e)
                pass
            self.aTime = time.time()
            if self.aTime > (self.TrackTime+self.TrackUpdateTime):
                self.TrackUpdateTime = time.time()
                self.resetTracker = True

        if len(self.cvFrame) > 0:
            cv.putText(self.cvFrame, f'Velocity: {int(self.velocityObject)}', (40, 70), cv.FONT_HERSHEY_PLAIN,
                    3, (255, 0, 0), 3)
            cv.namedWindow("IMAGE_FROM_OBJECT_POS", cv.WINDOW_NORMAL)
            cv.moveWindow("IMAGE_FROM_OBJECT_POS", 640, 0)
            cv.imshow("IMAGE_FROM_OBJECT_POS", self.cvFrame)
            cv.resizeWindow("IMAGE_FROM_OBJECT_POS", 640, 480)
            cv.waitKey(1)

    def restart_Tracker(self) -> None:
        self.statusNode = False
        self.TRACKER = OPENCV_OBJECT_TRACKERS["csrt"]()
        self.velocityObject = 0
        self.statusTracking = False
        self.resetTracker = False
        rospy.loginfo("Tracker Reset.")
        self.dataPosReady = False
        self.posObject = ()
        self.borderBoxes = []
        self.TrackUpdateTime = time.time()


def main():
    system('clear')
    time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    #rate = rospy.Rate(5.0)
    """Inicializar el objeto object_pos"""
    objNode = ObjectPos()
    objNode.subTopicBorderBoxesName = TOPIC_S1_NAME
    objNode.subTopicImageName = TOPIC_S2_NAME
    objNode.pubTopicStatusNodeName = TOPIC_P1_NAME
    objNode.pubTopicDataNodeName = TOPIC_P2_NAME
    objNode.start_subscribers()
    objNode.start_publishers()
    time.sleep(1)
    while not rospy.is_shutdown():
        if objNode.dataReceivedTopic1 == False and objNode.dataReceivedTopic2 == False:
            print("[WARNING] Datos no recibidos")
        else:
            objNode.get_pos()
            objNode.process_img()
        objNode.pubTopicStatusNode.publish(objNode.statusNode)
        objNode.pubTopicDataNode.publish(objNode.data)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass