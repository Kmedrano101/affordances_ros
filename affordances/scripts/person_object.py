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

# Clase


class PersonObject:
    """Person object"""

    def __init__(self):
        self._subTopicBoundingBoxesName = ""		# Private and external access
        self._subTopicFoundObjectsName = ""			# Private and external access
        self._pubTopicFlag = None					# Private and external access
        self._pubTopicObjectBoxes = None			# Private and external access
        self._pubTopicObjectBoxesName = ""			# Private and external access
        self._pubTopicFlagName = ""					# Private and external access
        self._flagSet = False						# Only Private no external access
        self._nobjects = 0							# Only Private no external access
        self._bounding_boxes = []					# Only Private no external access
        self._dataReceivedTopic1 = False			# Private and external access
        self._dataReceivedTopic2 = False			# Private and external access

    """Propierties"""
    @property
    def subTopicBoundingBoxesName(self):
        """The subTopicBoundingBoxesName property."""
        return self._subTopicBoundingBoxesName

    @subTopicBoundingBoxesName.setter
    def subTopicBoundingBoxesName(self, value):
        self._subTopicBoundingBoxesName = value

    @property
    def subTopicFoundObjectsName(self):
        """The subTopicFoundObjectsName property."""
        return self._subTopicFoundObjectsName

    @subTopicFoundObjectsName.setter
    def subTopicFoundObjectsName(self, value):
        self._subTopicFoundObjectsName = value

    @property
    def pubTopicObjectBoxesName(self):
        """The pubTopicObjectBoxesName property."""
        return self._pubTopicObjectBoxesName

    @pubTopicObjectBoxesName.setter
    def pubTopicObjectBoxesName(self, value):
        self._pubTopicObjectBoxesName = value

    @property
    def pubTopicFlag(self):
        """The pubTopicFlag property."""
        return self._pubTopicFlag

    @pubTopicFlag.setter
    def pubTopicFlag(self, value):
        self._pubTopicFlag = value

    @property
    def pubTopicFlagName(self):
        """The pubTopicFlagName property."""
        return self._pubTopicFlagName

    @pubTopicFlagName.setter
    def pubTopicFlagName(self, value):
        self._pubTopicFlagName = value

    @property
    def pubTopicObjectBoxes(self):
        """The pubTopicObjectBoxes property."""
        return self._pubTopicObjectBoxes

    @pubTopicObjectBoxes.setter
    def pubTopicObjectBoxes(self, value):
        self._pubTopicObjectBoxes = value

    @property
    def flagSet(self):
        """The flagSet property."""
        return self._flagSet

    @flagSet.setter
    def flagSet(self, value):
        self._flagSet = value

    @property
    def nobjects(self):
        """The nobjects property."""
        return self._nobjects

    @nobjects.setter
    def nobjects(self, value):
        self._nobjects = value

    """ Methods and Actions"""

    def bouding_boxes_callback(self, msg) -> None:
        self._dataReceivedTopic1 = True
        self._bounding_boxes = msg
        #rospy.loginfo("CallBack Bounding Boxes")
        for num, box in enumerate(self._bounding_boxes.bounding_boxes, start=1):
            rospy.loginfo(f"Nombre de objeto {num}: {box.Class}")

    def number_objects_callback(self, msg) -> None:
        self._dataReceivedTopic2 = True
        self._nobjects = msg.count
        #rospy.loginfo(f"Numero de objetos detectados: {self._nobjects}")

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicBoundingBoxesName, BoundingBoxes, self.bouding_boxes_callback)
        rospy.Subscriber(self._subTopicFoundObjectsName, ObjectCount, self.number_objects_callback)

    def start_publishers(self) -> None:
        self._pubTopicFlag = rospy.Publisher(
            self._pubTopicFlagName, Bool, queue_size=10)
        self._pubTopicObjectBoxes = rospy.Publisher(
            self._pubTopicObjectBoxesName, BoundingBoxes, queue_size=10)

    def get_state_flag(self) -> Bool:  # Posible validacion del objeto con nube de puntos
        """Retorna verdadero cuando se decta dos o mas objetos con la condicion de que uno de los objetos tiene que ser 'person'"""
        self._flagSet = False
        if self._nobjects >= 2:
            for box in self._bounding_boxes.bounding_boxes:
                if box.Class == "person":
                    self._flagSet = True
        return self._flagSet

    def get_object_boxes(self) -> BoundingBoxes:
        """Retorna posicion del un objeto particular"""
        boxes = self._bounding_boxes
        #boxes.bounding_boxes = []
        for i, box in enumerate(boxes.bounding_boxes):
            if box.Class == "person":  # Probablemente listar los objetos de interes
                del boxes.bounding_boxes[i]
        return boxes


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
    """Inicializar el objeto PersonObject"""
    objNode = PersonObject()
    objNode.subTopicBoundingBoxesName = TOPIC_S1_NAME
    objNode.subTopicFoundObjectsName = TOPIC_S2_NAME
    objNode.pubTopicObjectBoxesName = TOPIC_P1_NAME
    objNode.pubTopicFlagName = TOPIC_P2_NAME
    objNode.start_subscribers()
    objNode.start_publishers()
    time.sleep(2)
    while not rospy.is_shutdown():
        if objNode._dataReceivedTopic1 == False or objNode._dataReceivedTopic2 == False:
            print("[WARNING] Datos no recibidos del paquete darknet_ros")
        else:
            value = objNode.get_state_flag()
            objNode.pubTopicObjectBoxes.publish(objNode.get_object_boxes())
            objNode.pubTopicFlag.publish(value)
        # rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
