#!/usr/bin/env python3

# Importar modulos
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
from affordances.msg import objectData
from affordances.msg import personPos
import time
import os
import csv
from sklearn.preprocessing import StandardScaler
import numpy as np

import pyttsx3
engine = pyttsx3.init(driverName='espeak')
rate = engine.getProperty('rate')
engine.setProperty('rate', rate-50)


#LISTVOICE = []

# FILE DATASET
CSV_FILE = open('/home/kmedrano101/catkin_ws/src/affordances/data/dataset.csv', 'a')
# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_agent_name", default="agent")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_object_pos/topic", default="/affordances/data_object_pos")
TOPIC_S2_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_person_pose/topic", default="/affordances/data_person_pose")
TOPIC_S3_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_object_pos/topic", default="/affordances/status_object_pos")
TOPIC_S4_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_person_pose/topic", default="/affordances/status_person_pose")


# Clase

class train_ML:
    """train M.L. object"""

    def __init__(self):
        self._dataPersonPose = personPos()		# Only Private no external access
        self._dataObjectPose = objectData()		# Only Private no external access
        self._statusPose = False	            # Only Private no external access
        self._statusObject = False	            # Only Private no external access
        self._subTopicPersonPoseName = None	    # Only Private no external access
        self._subTopicObjectPoseName = None	    # Only Private no external access
        self._subTopicStatusPoseName = None	    # Only Private no external access
        self._subTopicStatusObjectName = None	# Only Private no external access
        self._FlagCaptureData = 0
        self.sTime = 0.85
        self.iTime = 0
        self.aTime = time.time()
        self.VectorData = []
        self.Counter = 0
        self.scaler = StandardScaler()
        self.ResetCounterCap = False
    """ Properties """
    
    @property
    def dataPersonPose(self):
        """The dataPersonPose property."""
        return self._dataPersonPose
    @dataPersonPose.setter
    def dataPersonPose(self, value):
        self._dataPersonPose = value

    @property
    def dataObjectPose(self):
        """The dataObjectPose property."""
        return self._dataObjectPose

    @dataObjectPose.setter
    def dataObjectPose(self, value):
        self._dataObjectPose = value

    @property
    def statusPose(self):
        """The statusPose property."""
        return self._statusPose

    @statusPose.setter
    def statusPose(self, value):
        self._statusPose = value

    @property
    def statusObject(self):
        """The statusObject property."""
        return self._statusObject

    @statusObject.setter
    def statusObject(self, value):
        self._statusObject = value

    @property
    def subTopicPersonPoseName(self):
        """The subTopicPersonPoseName property."""
        return self._subTopicPersonPoseName

    @subTopicPersonPoseName.setter
    def subTopicPersonPoseName(self, value):
        self._subTopicPersonPoseName = value    

    @property
    def subTopicObjectPoseName(self):
        """The subTopicObjectPoseName property."""
        return self._subTopicObjectPoseName

    @subTopicObjectPoseName.setter
    def subTopicObjectPoseName(self, value):
        self._subTopicObjectPoseName = value

    @property
    def subTopicStatusPoseName(self):
        """The subTopicStatusPoseName property."""
        return self._subTopicStatusPoseName

    @subTopicStatusPoseName.setter
    def subTopicStatusPoseName(self, value):
        self._subTopicStatusPoseName = value

    @property
    def subTopicStatusObjectName(self):
        """The subTopicStatusObjectName property."""
        return self._subTopicStatusObjectName

    @subTopicStatusObjectName.setter
    def subTopicStatusObjectName(self, value):
        self._subTopicStatusObjectName = value  

    """ Methods and Actions"""

    def data_pose_callback(self, msg) -> None:
        self._dataPersonPose = msg
        #print("DataPose",self._dataPersonPose)

    def data_object_callback(self, msg) -> None:
        self._dataObjectPose = msg
        #print("DataObject",self._dataObjectPose)

    def status_pose_callback(self, msg) -> None:
        self._statusPose = msg.data
        #print("Data status pose",self.statusPose)

    def status_object_callback(self, msg) -> None:
        self._statusObject = msg.data
        #print("Data status object",self.statusObject)

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicPersonPoseName, personPos, self.data_pose_callback)
        rospy.Subscriber(self._subTopicObjectPoseName, objectData, self.data_object_callback)
        rospy.Subscriber(self._subTopicStatusPoseName, Bool, self.status_pose_callback)
        rospy.Subscriber(self._subTopicStatusObjectName, Bool, self.status_object_callback)

    def save_data_set(self) -> None:
        # Considerar descartar los datos en caso de no continuidad correcta de las mismas
        # Lista de Puntos para POSE de interes
        PosePoints = [0,11,12,13,14,15,16,23,24,25,26,27,28] # Realizar analisis de reduccion de dimensionalidad
        temp = []
        temp2 = []
        if self._statusObject == True and self._statusPose == True:
            self.ResetCounterCap = True
            self.aTime = time.time()
            if self.aTime > (self.sTime+self.iTime):
                print("Capturing Data...") 
                self.iTime = time.time()
                if self.Counter <= 5:
                    # Guardando datos de Human Pose
                    for pose in self._dataPersonPose.poses:
                        if pose.id_pos in PosePoints:
                            temp += [pose.position.x, pose.position.y, pose.position.z, pose.visibility]
                    # Agragando datos de Pose Object // Posibilidad de agregar distancia objeto a puntos claves del cuerpo
                    classObject = 1 if self.dataObjectPose.classObject == "laptop" else 3
                    temp2 = [self.dataObjectPose.x,self.dataObjectPose.y, self.dataObjectPose.velocity]
                    temp2 = np.array(temp2)
                    temp2 = self.scaler.fit_transform(temp2[:,np.newaxis]).tolist()
                    temp2 = [temp2[0][0],temp2[1][0],temp2[2][0],classObject]
                    self.VectorData.extend(temp)
                    self.VectorData.extend(temp2)
                    self.Counter += 1
                else:
                    value = input("Save data? y/n: ")
                    if value == "y":    
                        writer = csv.writer(CSV_FILE, delimiter=':', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        writer.writerow(self.VectorData)
                        CSV_FILE.flush()
                        os.fsync(CSV_FILE.fileno())
                        rospy.loginfo("Saving Data ...")
                        self.Counter = 0
                        self.VectorData = []
                        self.iTime = time.time()
                    else:
                        print("Data does not save!")
                        self.Counter = 0
                print("Counter:",self.Counter)
        if self.ResetCounterCap and self._statusObject == False:
            self.Counter = 0
            self.VectorData = []

## Actualizar captura de datos si se pierde la secuencia en caso de seguimiento de objeto

def main():
    # Don't forget to remove this test mode
    os.system('clear')
    time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    #rate = rospy.Rate(1.0)
    """Inicializar el objeto train_ML"""
    objNode = train_ML()
    objNode.subTopicObjectPoseName = TOPIC_S1_NAME
    objNode.subTopicPersonPoseName = TOPIC_S2_NAME
    objNode.subTopicStatusObjectName = TOPIC_S3_NAME
    objNode.subTopicStatusPoseName = TOPIC_S4_NAME
    objNode.start_subscribers()
    #engine.say("I am a robot")
    #engine.runAndWait()
    time.sleep(3)
    INI = 0
    while not rospy.is_shutdown():
        if INI == 0:
            print(f"OPCIONES DE {NODE_NAME} NODE: \n\t1.- MODO APRENDIZAJE (CAPTURA DE DATOS)\n\t2.- MODO IDENTIFICAR")
            op = input("Opcion: ")
            INI = 1
        if op == "1":
            objNode.save_data_set()
        if op == "2":
            pass
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
