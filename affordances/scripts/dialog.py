#!/usr/bin/env python3

# Importar modulos
import rospy
from rospy.exceptions import ROSException
import time
from os import system
from std_msgs.msg import Int8
from affordances.msg import affordance

import pyttsx3
engine = pyttsx3.init(driverName='espeak')
rate = engine.getProperty('rate')
engine.setProperty('rate', rate-50)

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_dialog_name", default="dialog")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/utility/topic", default="/affordances/utilities")

# Clase Dialog


class Dialog:
    """Dialog"""

    def __init__(self):
        self._stateNode = None
        self._subTopicUtilityName = None
        self.subTopicUtility = None
        self.objectUtility = None
        self.utilities = None
        self.affordance = affordance()

    """Properties"""

    @property
    def subTopicUtilityName(self):
        """The subTopicBoundingBoxesName property."""
        return self._subTopicUtilityName

    @subTopicUtilityName.setter
    def subTopicUtilityName(self, value):
        if value:
            self._subTopicUtilityName = value
        else:
            rospy.loginfo("Invalid Name of topic Utility")

    """ Methods and Actions"""
    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicActivityName, affordance, self.utility_callback)

    def utility_callback(self, msg) -> None:
        self.affordance = msg
        print("Data: ",self.affordance)

    def speakUtility(self) ->None:
        pass

def main():
    system('clear')
    time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    """Inicializar el objeto object_pos"""
    objNode = Dialog()
    objNode.subTopicUtilityName = TOPIC_S1_NAME
    objNode.start_subscribers()
    #engine.say("Node Dialog")
    #engine.runAndWait()
    time.sleep(1)
    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
