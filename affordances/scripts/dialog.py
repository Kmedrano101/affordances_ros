#!/usr/bin/env python3

# Importar modulos
import rospy
from rospy.exceptions import ROSException
import time
from os import system
from std_msgs.msg import Int8

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
    PACKAGE_NAME+"publishers/type_activity/topic", default="/affordances/number_activity")

# Clase Dialog


class Dialog:
    """Dialog"""

    def __init__(self):
        self._numberActivity = Int8()				# Only Private no external access
        self._stateNode = None                      # Private and external access
        self._subTopicActivityName = None           # Private and external access

    """Properties"""
    
    """ Methods and Actions"""
    def speakActivity(self) ->None:
        pass


    def number_activity_callback(self, msg) -> None:
        self._numberActivity = msg

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicActivityName, int, self.number_activity_callback)


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
    objNode = Dialog()
    objNode._subTopicActivityName = TOPIC_S1_NAME
    objNode.start_subscribers()
    #engine.say("Node Dialog")
    #engine.runAndWait()
    time.sleep(2)
    while not rospy.is_shutdown():
        pass
        #rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
