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
# Test these objects tracker
BRIDGE = CvBridge()

# Clase

class ObjectPos:
    """Person object"""

    def __init__(self):
        self._objectPos = None					# Only Private no external access
        self._cvFrame = []						# Only Private no external access
        self._subTopicBorderBoxesName = None 	# Private and external access
        self._subTopicImageName = None 			# Private and external access
        self._pubTopicStatusNodeName = None		# Private and external access
        self._pubTopicDataNodeName = None	    # Private and external access
        self._pubTopicStatusNode = None			# Private and external access
        self._pubTopicDataNode = None	        # Private and external access
        self._statusNode = False				# Private and external access
        self._borderBoxes = ()					# Only Private no external access
        self._dataReceivedTopic1 = False		# Private and external access
        self._dataReceivedTopic2 = False		# Private and external access
        self._dataPosReady = False				# Only Private no external access
        self._posObject = ()					# Only Private no external access
        self._sTime = 0.1
        self._iTime = 0
        self._fValue = 0
        self._X1 = 0
        self._Y1 = 0
        self._ListVelocities = []
        self._velocityObject = 0
        self.data = objectData()
        self.statusTracking = False
        self.resetTracker = False
        self.auxReset = False
        self.TRACKER = OPENCV_OBJECT_TRACKERS["kcf"]()

    """Propierties"""
    @property
    def subTopicBorderBoxesName(self):
        """The subTopicBorderBoxesName property."""
        return self._subTopicBorderBoxesName

    @subTopicBorderBoxesName.setter
    def subTopicBorderBoxesName(self, value):
        self._subTopicBorderBoxesName = value

    @property
    def subTopicImageName(self):
        """The subTopicImageName property."""
        return self._subTopicImageName

    @subTopicImageName.setter
    def subTopicImageName(self, value):
        self._subTopicImageName = value

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
        """The pubTopicDataNode property."""
        return self._pubTopicDataNode

    @pubTopicDataNode.setter
    def pubTopicDataNode(self, value):
        self._pubTopicDataNode = value

    @property
    def pubTopicStatusNode(self):
        """The pubTopicStatusNode property."""
        return self._pubTopicStatusNode

    @pubTopicStatusNode.setter
    def pubTopicStatusNode(self, value):
        self._pubTopicStatusNode = value

    @property
    def statusNode(self):
        """The statusNode property."""
        return self._statusNode

    @statusNode.setter
    def statusNode(self, value):
        self._statusNode = value

    @property
    def sTime(self):
        """The sTime property."""
        return self._sTime

    @sTime.setter
    def sTime(self, value):
        self._sTime = value

    @property
    def iTime(self):
        """The iTime property."""
        return self._iTime

    @iTime.setter
    def iTime(self, value):
        self._iTime = value

    @property
    def fValue(self):
        """The fValue property."""
        return self._fValue

    @fValue.setter
    def fValue(self, value):
        self._fValue = value    

    @property
    def X1(self):
        """The X1 property."""
        return self._X1

    @X1.setter
    def X1(self, value):
        self._X1 = value

    @property
    def Y1(self):
        """The Y1 property."""
        return self._Y1

    @Y1.setter
    def Y1(self, value):
        self._Y1 = value

    @property
    def ListVelocities(self):
        """The ListVelocities property."""
        return self._ListVelocities

    @ListVelocities.setter
    def ListVelocities(self, value):
        self._ListVelocities = value

    @property
    def velocityObject(self):
        """The velocityObject property."""
        return self._velocityObject

    @velocityObject.setter
    def velocityObject(self, value):
        self._velocityObject = value

    """ Methods and Actions"""

    def image_source_callback(self, msg) -> None:
        self._dataReceivedTopic1 = True
        try:
            self._cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ERROR] {e}")

    def border_boxes_callback(self, msg) -> None:
        self._dataReceivedTopic2 = True
        self._borderBoxes = msg.bounding_boxes

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicImageName, Image, self.image_source_callback)
        rospy.Subscriber(self._subTopicBorderBoxesName, BoundingBoxes, self.border_boxes_callback)

    def start_publishers(self) -> None:
        self._pubTopicStatusNode = rospy.Publisher(
            self._pubTopicStatusNodeName, Bool, queue_size=10)
        self._pubTopicDataNode = rospy.Publisher(self._pubTopicDataNodeName, objectData, queue_size=10)

    def get_pos(self) -> None:
        """ Funcion para obtener la posicion central del objeto a realizar el tracking"""
        pos = []
        self._dataPosReady = False
        
        if self._dataReceivedTopic2 and not self.statusTracking:
            for box in self._borderBoxes:
                if box.Class == "cup":  # Considerar varios objetos del mismo tipo IMPORTANT
                    #rospy.loginfo("Object CUP encontrado.")
                    #print(f"Data ({box.xmin},{box.ymin}, {box.xmax}, {box.ymax})")
                    pos.append(box.xmin-1)
                    pos.append(box.ymin-1)
                    pos.append(abs(box.ymax - box.ymin))
                    pos.append(abs(box.xmax - box.xmin)+1)
                    self._dataPosReady = True
                    rospy.loginfo("Object detected")
                    self._posObject = tuple(pos)
                    self._borderBoxes = []
                    break

    def find_velocity(self,x,y) -> None:
        if self.fValue == 0:
            self.X1, self.Y1 = x, y
            self.fValue = 1
        self.aTime = time.time()
        if self.aTime > (self.sTime+self.iTime):
            X2, Y2 = x, y
            V = abs(self.X1 - X2) + abs(self.Y1 - Y2)
            self.ListVelocities.append(V)
            if len(self.ListVelocities) >= 5:
                self.velocityObject = (sum(self.ListVelocities)/len(self.ListVelocities)) * 2 # FPS In this case is just per 1s
                self.ListVelocities = []
            self.fValue = 0
            self.iTime = time.time()

    def process_img(self) -> None:
        # Verificar y validar datos recibidos enfocar seguimiento de objeto
        #success = False
        if self.resetTracker and not self.auxReset:
                self.restart_Tracker()
                self.auxReset = True
        if self._dataPosReady and self._dataReceivedTopic1 and not self._statusNode:
            self.statusTracking = True
            if self._posObject:
                self.TRACKER.init(self._cvFrame, self._posObject)
                rospy.loginfo("Tracker started.")
                self.auxReset = False
            self._statusNode = True
        if self.statusTracking:
            try:
                (success, box) = self.TRACKER.update(self._cvFrame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv.rectangle(self._cvFrame, (x, y), (x + h, y + w), (0, 255, 0), 2)
                    X = int((x*2+h)/2)
                    Y = int((y*2+w)/2)
                    self.find_velocity(X,Y)
                    self.data.x = X
                    self.data.y = Y
                    self.data.velocity = self.velocityObject
                    cv.circle(self._cvFrame, (X,Y), 4, (255, 0, 0), -1)
                else:
                    self.resetTracker = True
            except Exception as e:
                print("Error updating IMG", e)
                pass
        if len(self._cvFrame)>0:
            cv.putText(self._cvFrame, f'Velocity: {int(self.velocityObject)}', (40, 70), cv.FONT_HERSHEY_PLAIN,
                    3, (255, 0, 0), 3)
            cv.imshow("Image from Node ObjectPos", self._cvFrame)
            cv.waitKey(1)

    def restart_Tracker(self) -> None:
        self._statusNode = False
        #self.TRACKER.release() 
        self.TRACKER = OPENCV_OBJECT_TRACKERS["kcf"]()
        self._velocityObject = 0
        self.statusTracking = False
        self.resetTracker = False
        rospy.loginfo("Tracker Reset.")
        self._dataPosReady = False
        self._posObject = ()
        self._borderBoxes = []
        #self._cvFrame = []	


def main():
    # Don't forget to remove this test mode
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
    time.sleep(2)
    while not rospy.is_shutdown():
        # or objNode._dataReceivedTopic2 == False:
        if objNode._dataReceivedTopic1 == False and objNode._dataReceivedTopic2 == False:
            print("[WARNING] Datos no recibidos")
        else:
            objNode.get_pos()
            objNode.process_img()
        objNode.pubTopicStatusNode.publish(objNode.statusNode)
        objNode.pubTopicDataNode.publish(objNode.data)
        #rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
