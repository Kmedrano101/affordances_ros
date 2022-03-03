#!/usr/bin/env python3

# Importar modulos
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
from affordances.msg import objectData
from affordances.msg import personPos
import time
from os import system
import csv
from sklearn.preprocessing import StandardScaler
import numpy as np

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_train_name", default="train_ML")
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
        self.sTime = 1.5
        self.iTime = 0
        self.aTime = time.time()
        self.VectorData = []
        self.Counter = 0
        self.scaler = StandardScaler()
        
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
            #rospy.loginfo("Listo para capturar Datos")
            self.aTime = time.time()
            if self.aTime > (self.sTime+self.iTime):
                print("Capturing Data...") 
                self.iTime = time.time()
                if self.Counter <= 5:
                    # Guardando datos de Human Pose
                    #print(type(self._dataPersonPose))
                    #print(self._dataPersonPose)
                    for pose in self._dataPersonPose.poses:
                        if pose.id_pos in PosePoints:
                            temp += [pose.position.x, pose.position.y, pose.position.z, pose.visibility]
                    # Agragando datos de Pose Object
                    temp2 = [self.dataObjectPose.x,self.dataObjectPose.y, self.dataObjectPose.velocity]
                    temp2 = np.array(temp2)
                    temp2 = self.scaler.fit_transform(temp2[:,np.newaxis]).tolist()
                    temp2 = [temp2[0][0],temp2[1][0],temp2[2][0]]
                    self.VectorData.append(temp)
                    self.VectorData.append(temp2)
                    self.Counter += 1
                else:
                    print(self.VectorData[0])
                    with open('dataset.csv', 'a') as f:
                        write = csv.writer(f,delimiter=':', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        write.writerow(self.VectorData[0])
                        rospy.loginfo("Saving Data ...")
                    self.VectorData = []
                    self.Counter = 0
                print("Counter:",self.Counter)

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
    """Inicializar el objeto train_ML"""
    objNode = train_ML()
    objNode.subTopicObjectPoseName = TOPIC_S1_NAME
    objNode.subTopicPersonPoseName = TOPIC_S2_NAME
    objNode.subTopicStatusObjectName = TOPIC_S3_NAME
    objNode.subTopicStatusPoseName = TOPIC_S4_NAME
    objNode.start_subscribers()
    time.sleep(3)
    while not rospy.is_shutdown():
        objNode.save_data_set()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass