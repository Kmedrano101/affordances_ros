#!/usr/bin/env python3

# Importar modulos
from xmlrpc.client import Boolean
import rospy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
import time
from os import system

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(
    PACKAGE_NAME+"node_po_name", default="person_object")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/object_bounding_boxes/topic", default="/darknet_ros/bounding_boxes")
TOPIC_S2_NAME = rospy.get_param(
    PACKAGE_NAME+"subscribers/number_object/topic", default="/darknet_ros/found_object")
TOPIC_P1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/bounding_boxes/topic", default="/affordances/object_bounding_boxes")
TOPIC_P2_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/flag/topic", default="/affordances/flag")

# Clase PersonObject


class PersonObject:
    """Person object"""

    def __init__(self):
        self._subTopicBoundingBoxesName = ""
        self._subTopicFoundObjectsName = ""
        self._pubTopicObjectBoxesName = ""
        self._pubTopicFlagName = ""
        self.pubTopicFlag = None
        self.pubTopicObjectBoxes = None
        self.flagSet = False
        self.nobjects = 0
        self.bounding_boxes = []
        self.dataReceivedTopic1 = False
        self.dataReceivedTopic2 = False

    """Propierties"""
    @property
    def subTopicBoundingBoxesName(self):
        """The subTopicBoundingBoxesName property."""
        return self._subTopicBoundingBoxesName

    @subTopicBoundingBoxesName.setter
    def subTopicBoundingBoxesName(self, value):
        if value:
            self._subTopicBoundingBoxesName = value
        else:
            rospy.loginfo("Invalid Name of topic BoundingBoxes")

    @property
    def subTopicFoundObjectsName(self):
        """The subTopicFoundObjectsName property."""
        return self._subTopicFoundObjectsName

    @subTopicFoundObjectsName.setter
    def subTopicFoundObjectsName(self, value):
        if value:
            self._subTopicFoundObjectsName = value
        else:
            rospy.loginfo("Invalid Name of topic FoundObjects")

    @property
    def pubTopicObjectBoxesName(self):
        """The pubTopicObjectBoxesName property."""
        return self._pubTopicObjectBoxesName

    @pubTopicObjectBoxesName.setter
    def pubTopicObjectBoxesName(self, value):
        if value:
            self._pubTopicObjectBoxesName = value
        else:
            rospy.loginfo("Invalid Name of topic ObjectBoxes")

    @property
    def pubTopicFlagName(self):
        """The pubTopicFlagName property."""
        return self._pubTopicFlagName

    @pubTopicFlagName.setter
    def pubTopicFlagName(self, value):
        if value:
            self._pubTopicFlagName = value
        else:
            rospy.loginfo("Invalid Name of topic Flag")

    """ Methods and Actions"""

    def bouding_boxes_callback(self, msg) -> None:
        self.dataReceivedTopic1 = True
        self.bounding_boxes = msg
        #rospy.loginfo("CallBack Bounding Boxes")
        for num, box in enumerate(self.bounding_boxes.bounding_boxes, start=1):
            rospy.loginfo(f"Nombre de objeto {num}: {box.Class}")

    def number_objects_callback(self, msg) -> None:
        self.dataReceivedTopic2 = True
        self.nobjects = msg.count

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicBoundingBoxesName,
                        BoundingBoxes, self.bouding_boxes_callback)
        rospy.Subscriber(self._subTopicFoundObjectsName,
                        ObjectCount, self.number_objects_callback)

    def start_publishers(self) -> None:
        self.pubTopicFlag = rospy.Publisher(
            self._pubTopicFlagName, Bool, queue_size=10)
        self.pubTopicObjectBoxes = rospy.Publisher(
            self._pubTopicObjectBoxesName, BoundingBoxes, queue_size=10)

    def get_state_flag(self) -> Bool:
        """Retorna verdadero cuando se decta dos o mas objetos 
        con la condicion de que uno de los objetos tiene que ser 'person'"""
        self.flagSet = False
        if self.nobjects >= 2:
            for box in self.bounding_boxes.bounding_boxes:
                if box.Class == "person":
                    self.flagSet = True
        return self.flagSet

    def get_object_boxes(self) -> BoundingBoxes:
        """Retorna posicion del un objeto particular"""
        boxes = self.bounding_boxes
        #boxes.bounding_boxes = []
        for i, box in enumerate(boxes.bounding_boxes):
            if box.Class == "person":
                del boxes.bounding_boxes[i]
        return boxes


def main():
    system('clear')
    time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    #rate = rospy.Rate(1.0)
    """Inicializar el objeto PersonObject"""
    objNode = PersonObject()
    objNode.subTopicBoundingBoxesName = TOPIC_S1_NAME
    objNode.subTopicFoundObjectsName = TOPIC_S2_NAME
    objNode.pubTopicObjectBoxesName = TOPIC_P1_NAME
    objNode.pubTopicFlagName = TOPIC_P2_NAME
    objNode.start_subscribers()
    objNode.start_publishers()
    time.sleep(1)
    INFO1 = True
    while not rospy.is_shutdown():
        if (not objNode.dataReceivedTopic1  or not objNode.dataReceivedTopic2) and INFO1:
            print("[WARNING] Datos no recibidos del paquete darknet_ros")
            INFO1 = False
        elif objNode.bounding_boxes:
            value = objNode.get_state_flag()
            objNode.pubTopicObjectBoxes.publish(objNode.get_object_boxes())
            objNode.pubTopicFlag.publish(value)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
