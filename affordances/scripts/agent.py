#!/usr/bin/env python3

# Importar modulos
from cv2 import sqrt
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
from affordances.msg import personPos, objectData, affordance
import time
import os
import csv
from sklearn.preprocessing import StandardScaler
import numpy as np
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', type=int,
                    help='1 Trainging Mode - 2 Prediction Mode', default=1)

# al usar join path se enlaza la direccion por defecto de ros (.ros) validar quitar /.ros
args = parser.parse_args()
# os.path.join(os.getcwd(),"catkin_ws/src/affordances/data/")
path = "/home/kmedrano101/catkin_ws/src/affordances/data/"

# Parametros y variables
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = "/affordances/"
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_agent_name", default="agent")
TOPIC_S1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_object_pos/topic", default="/affordances/data_object_pos")
TOPIC_S2_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/data_person_pose/topic", default="/affordances/data_person_pose")
TOPIC_S3_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_object_pos/topic", default="/affordances/status_object_pos")
TOPIC_S4_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/status_person_pose/topic", default="/affordances/status_person_pose")
TOPIC_P1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/utility/topic", default="/affordances/utilities")

DATASET_NAME = rospy.get_param(
    PACKAGE_NAME+"dataset_name", default="dataset.csv")
MODEL_NAME = rospy.get_param(PACKAGE_NAME+"model_name", default="model.sav")
DETECTION_OBJECTS = rospy.get_param(PACKAGE_NAME+"detection_objects")
VALOR_Y = rospy.get_param(PACKAGE_NAME+"valor_y")

try:
    # FILE DATASET CON DISTANCIAS Y VELOCIDAD
    CSV_FILE_1 = open(path+'DSAllPoints_B.csv', 'a')
    CSV_FILE_2 = open(path+'DS13Points_B.csv', 'a')
    CSV_FILE_3 = open(path+'DS9Points_B.csv', 'a')
    # FILE DATASET SIN DISTANCIAS Y VELOCIDAD
    CSV_FILE_4 = open(path+'DSAllPoints_C.csv', 'a')
    CSV_FILE_5 = open(path+'DS13Points_C.csv', 'a')
    CSV_FILE_6 = open(path+'DS9Points_C.csv', 'a')
    CSV_FILE = open(path+DATASET_NAME, 'a')
except:
    print("Error al leer los archivos csv")
# Class Agent


class Agent:
    """Agent M.L. object"""

    def __init__(self):
        self._subTopicPersonPoseName = None
        self._subTopicObjectPoseName = None
        self._subTopicStatusPoseName = None
        self._subTopicStatusObjectName = None
        self._pubTopicAffordancesName = None
        self.dataPersonPose = personPos()
        self.dataObjectPos = objectData()
        self.statusPose = False
        self.statusObject = False
        self.pubTopicAffordances = None
        self.FlagCaptureData = 0
        self.sTime = 0.95
        self.iTime = 0
        self.aTime = time.time()
        self.VectorData1 = []
        self.VectorData2 = []
        self.VectorData3 = []
        self.VectorData4 = []
        self.VectorData5 = []
        self.VectorData6 = []
        self.Counter = 0
        self.scaler = StandardScaler()
        self.ResetCounterCap = False
        self.numeroDatos = 0
        self.DetectedObjects = [0] * len(DETECTION_OBJECTS)

    """ Properties """
    @property
    def subTopicPersonPoseName(self):
        """The subTopicPersonPoseName property."""
        return self._subTopicPersonPoseName

    @subTopicPersonPoseName.setter
    def subTopicPersonPoseName(self, value):
        if value:
            self._subTopicPersonPoseName = value
        else:
            rospy.loginfo("Invalid Name of topic PersonPose")

    @property
    def subTopicObjectPoseName(self):
        """The subTopicObjectPoseName property."""
        return self._subTopicObjectPoseName

    @subTopicObjectPoseName.setter
    def subTopicObjectPoseName(self, value):
        if value:
            self._subTopicObjectPoseName = value
        else:
            rospy.loginfo("Invalid Name of topic ObjectPose")

    @property
    def subTopicStatusPoseName(self):
        """The subTopicStatusPoseName property."""
        return self._subTopicStatusPoseName

    @subTopicStatusPoseName.setter
    def subTopicStatusPoseName(self, value):
        if value:
            self._subTopicStatusPoseName = value
        else:
            rospy.loginfo("Invalid Name of topic StatusPoseName")

    @property
    def subTopicStatusObjectName(self):
        """The subTopicStatusObjectName property."""
        return self._subTopicStatusObjectName

    @subTopicStatusObjectName.setter
    def subTopicStatusObjectName(self, value):
        if value:
            self._subTopicStatusObjectName = value
        else:
            rospy.loginfo("Invalid Name of topic StatusObject")

    @property
    def pubTopicAffordancesName(self):
        """The subTopicStatusObjectName property."""
        return self._pubTopicAffordancesName

    @pubTopicAffordancesName.setter
    def pubTopicAffordancesName(self, value):
        if value:
            self._pubTopicAffordancesName = value
        else:
            rospy.loginfo("Invalid Name of topic Affordances")

    """ Methods and Actions"""

    def data_pose_callback(self, msg) -> None:
        self.dataPersonPose = msg

    def data_object_callback(self, msg) -> None:
        self.dataObjectPos = msg

    def status_pose_callback(self, msg) -> None:
        self.statusPose = msg.data

    def status_object_callback(self, msg) -> None:
        self.statusObject = msg.data

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicPersonPoseName,
                        personPos, self.data_pose_callback)
        rospy.Subscriber(self._subTopicObjectPoseName,
                        objectData, self.data_object_callback)
        rospy.Subscriber(self._subTopicStatusPoseName,
                        Bool, self.status_pose_callback)
        rospy.Subscriber(self._subTopicStatusObjectName,
                        Bool, self.status_object_callback)

    def start_publishers(self) -> None:
        self._pubTopicAffordances = rospy.Publisher(
            self._pubTopicAffordancesName, affordance, queue_size=10)

    def write_data_to_csv(self, file, data) -> None:
        try:
            writer = csv.writer(file, delimiter=':',
                                quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(data)
            file.flush()
            os.fsync(file.fileno())
        except Exception as e:
            print("Error: ", e)
            pass

    def reset_data(self) -> None:
        self.Counter = 0
        self.VectorData1 = []
        self.VectorData2 = []
        self.VectorData3 = []
        self.VectorData4 = []
        self.VectorData5 = []
        self.VectorData6 = []
        self.DetectedObjects = [0] * len(DETECTION_OBJECTS)

    def save_data(self) -> None:
        PosePoints = [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
        PosePoints2 = [0, 11, 12, 15, 16, 23, 24, 27, 28]
        for i in range(len(DETECTION_OBJECTS)):
            self.DetectedObjects[i] = 1 if DETECTION_OBJECTS[i] in self.dataObjectPos.objectsDetected else 0
        PosePointsALL = []
        PosePointsDistances = [0, 19, 20, 31, 32]
        temp1, temp2, temp3 = [], [], []
        distans = []
        if self.statusObject and not self.ResetCounterCap:
            # Con objeto de interes o sin objeto
            self.DetectedObjects.append(1)
            self.ResetCounterCap = True
        elif not self.ResetCounterCap:
            self.DetectedObjects.append(0)
            self.ResetCounterCap = True
        if self.DetectedObjects[len(self.DetectedObjects)-1] == 0 and self.statusObject:
            self.reset_data()
            self.DetectedObjects.append(1)
        if self.DetectedObjects[len(self.DetectedObjects)-1] == 1 and not self.statusObject:
            self.reset_data()
            self.DetectedObjects.append(0)
        if self.statusPose == True:
            self.aTime = time.time()
            if self.aTime > (self.sTime+self.iTime):
                print("Capturing Data...")
                self.iTime = time.time()
                if self.Counter <= 5:
                    # Datos objeto
                    if self.statusObject:
                        # considerar sin velocidad
                        temp3 = [self.dataObjectPos.x,
                                self.dataObjectPos.y, self.dataObjectPos.velocity]
                        temp3 = np.array(temp3)
                        temp3 = self.scaler.fit_transform(
                            temp3[:, np.newaxis]).tolist()
                        temp3 = [temp3[0][0], temp3[1][0], temp3[2][0]]
                    else:
                        temp3 = [0]*3
                    # Guardando datos de Human Pose
                    for pose in self.dataPersonPose.poses:
                        # Tomando las distancias
                        if pose.id_pos in PosePointsDistances and self.statusObject:
                            value = np.sqrt(
                                (temp3[0]-float(pose.position.x))**2+(temp3[1]-float(pose.position.y))**2)
                            distans.append(value)
                        # Tomando todas las Pose
                        PosePointsALL += [pose.position.x, pose.position.y,
                                        pose.position.z, pose.visibility]
                        # Tomando puntos de interes 1
                        if pose.id_pos in PosePoints:
                            temp1 += [pose.position.x, pose.position.y,
                                    pose.position.z, pose.visibility]
                        # Tomando puntos de interes 2
                        if pose.id_pos in PosePoints2:
                            temp2 += [pose.position.x, pose.position.y,
                                    pose.position.z, pose.visibility]
                    self.VectorData4.extend(PosePointsALL)
                    self.VectorData4.extend(temp3)
                    self.VectorData5.extend(temp1)
                    self.VectorData5.extend(temp3)
                    self.VectorData6.extend(temp2)
                    self.VectorData6.extend(temp3)
                    if not distans:
                        distans = [0] * len(PosePointsDistances)
                    temp3.extend(distans)
                    self.VectorData1.extend(PosePointsALL)
                    self.VectorData1.extend(temp3)
                    self.VectorData2.extend(temp1)
                    self.VectorData2.extend(temp3)
                    self.VectorData3.extend(temp2)
                    self.VectorData3.extend(temp3)
                    self.Counter += 1
                else:
                    self.DetectedObjects.append(VALOR_Y)
                    self.VectorData1.extend(self.DetectedObjects)
                    self.VectorData2.extend(self.DetectedObjects)
                    self.VectorData3.extend(self.DetectedObjects)
                    self.VectorData4.extend(self.DetectedObjects)
                    self.VectorData5.extend(self.DetectedObjects)
                    self.VectorData6.extend(self.DetectedObjects)
                    value = input("Save data? y/n: ")
                    if value == "y" and len(self.VectorData3) == 323:
                        error = 0
                        try:
                            self.write_data_to_csv(
                                CSV_FILE_1, self.VectorData1)
                            self.write_data_to_csv(
                                CSV_FILE_2, self.VectorData2)
                            self.write_data_to_csv(
                                CSV_FILE_3, self.VectorData3)
                            self.write_data_to_csv(
                                CSV_FILE_4, self.VectorData4)
                            self.write_data_to_csv(
                                CSV_FILE_5, self.VectorData5)
                            self.write_data_to_csv(
                                CSV_FILE_6, self.VectorData6)
                        except Exception as e:
                            print("Error al guardar csv", str(e))
                            error = 1
                            pass
                        if error == 0:
                            print("[INFO] Saving Data...")
                            self.numeroDatos += 1
                            print("NUMBER OF DATA: ", self.numeroDatos)
                    else:
                        print("[INFO] Data does not save!")
                    self.reset_data()
                    self.ResetCounterCap = False
                print("DATA Counter:", self.Counter)

    def prediction_mode(self) -> None:
        pass


def main():
    os.system('clear')
    # time.sleep(1)
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    """Inicializar el objeto Agent"""
    objNode = Agent()
    objNode.subTopicObjectPoseName = TOPIC_S1_NAME
    objNode.subTopicPersonPoseName = TOPIC_S2_NAME
    objNode.subTopicStatusObjectName = TOPIC_S3_NAME
    objNode.subTopicStatusPoseName = TOPIC_S4_NAME
    objNode.pubTopicAffordancesName = TOPIC_P1_NAME
    objNode.start_subscribers()
    objNode.start_publishers()
    time.sleep(3)
    if int(args.mode) == 1:
        print("Trainging mode")
    else:
        print("Prediction mode")
    while not rospy.is_shutdown():
        if args.mode == 1:
            objNode.save_data()
        else:
            objNode.prediction_mode()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass