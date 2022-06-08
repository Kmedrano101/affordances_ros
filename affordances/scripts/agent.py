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

args = parser.parse_args()

# FILE DATASET
CSV_FILE_1 = open(
    '/home/kmedrano101/catkin_ws/src/affordances/data/DSAllPoints.csv', 'a')
CSV_FILE_2 = open(
    '/home/kmedrano101/catkin_ws/src/affordances/data/DS13Points.csv', 'a')
CSV_FILE_3 = open(
    '/home/kmedrano101/catkin_ws/src/affordances/data/DS9Points.csv', 'a')
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
TOPIC_P1_NAME = rospy.get_param(
    PACKAGE_NAME+"publishers/utility/topic", default="/affordances/utilities")

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
        self.dataObjectPose = objectData()
        self.statusPose = False
        self.statusObject = False
        self.pubTopicAffordances = None
        self.FlagCaptureData = 0
        self.sTime = 0.95
        self.iTime = 0
        self.aTime = time.time()
        self.VectorData = []
        self.VectorData1 = []
        self.VectorData2 = []
        self.Counter = 0
        self.scaler = StandardScaler()
        self.ResetCounterCap = False
        self.numeroDatos = 0

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
        self.dataObjectPose = msg

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

    def save_data_set(self) -> None:
        # Lista de Puntos para POSE de interes
        # Realizar analisis de reduccion de dimensionalidad
        PosePoints = [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
        PosePoints2 = [0, 11, 12, 15, 16, 23, 24, 27, 28]
        PosePointsALL = []
        PosePointsDistances = [0, 19, 20, 31, 32]  # Distancias de interes
        temp, temp2 = [], []
        temp3, distans = [], []
        if self.statusObject == True and self.statusPose == True:
            self.ResetCounterCap = True
            self.aTime = time.time()
            if self.aTime > (self.sTime+self.iTime):
                print("Capturing Data...")
                self.iTime = time.time()
                if self.Counter <= 5:
                    # Datos objeto
                    temp3 = [self.dataObjectPose.x,
                            self.dataObjectPose.y, self.dataObjectPose.velocity]
                    temp3 = np.array(temp3)
                    temp3 = self.scaler.fit_transform(
                        temp3[:, np.newaxis]).tolist()
                    temp3 = [temp3[0][0], temp3[1][0], temp3[2][0]]
                    # Guardando datos de Human Pose
                    for pose in self.dataPersonPose.poses:
                        # Tomando las distancias
                        if pose.id_pos in PosePointsDistances:
                            value = np.sqrt(
                                (temp3[0]-float(pose.position.x))**2+(temp3[1]-float(pose.position.y))**2)
                            distans.append(value)
                        # Tomando todas las Pose
                        PosePointsALL += [pose.position.x, pose.position.y,
                                        pose.position.z, pose.visibility]
                        # Tomando puntos de interes 1
                        if pose.id_pos in PosePoints:
                            temp += [pose.position.x, pose.position.y,
                                    pose.position.z, pose.visibility]
                        # Tomando puntos de interes 2
                        if pose.id_pos in PosePoints2:
                            temp2 += [pose.position.x, pose.position.y,
                                    pose.position.z, pose.visibility]
                    temp3.extend(distans)
                    self.VectorData.extend(PosePointsALL)
                    self.VectorData.extend(temp3)
                    self.VectorData1.extend(temp)
                    self.VectorData1.extend(temp3)
                    self.VectorData2.extend(temp2)
                    self.VectorData2.extend(temp3)
                    self.Counter += 1
                else:
                    classObject = 4 if self.dataObjectPose.classObject == "bed" else 6
                    temp3.extend(["refrigerator", classObject])
                    self.VectorData.extend(temp3)
                    self.VectorData1.extend(temp3)
                    self.VectorData2.extend(temp3)
                    value = input("Save data? y/n: ")
                    if value == "y" and len(self.VectorData) == 842:
                        writer = csv.writer(
                            CSV_FILE_1, delimiter=':', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        writer.writerow(self.VectorData)
                        writer.writerow(self.VectorData1)
                        writer.writerow(self.VectorData2)
                        CSV_FILE_1.flush()
                        os.fsync(CSV_FILE_1.fileno())
                        CSV_FILE_2.flush()
                        os.fsync(CSV_FILE_2.fileno())
                        CSV_FILE_3.flush()
                        os.fsync(CSV_FILE_3.fileno())
                        print("[INFO] Saving Data...")
                        self.numeroDatos += 1
                        print("Numero Dato: ", self.numeroDatos)
                    else:
                        print("[INFO] Data does not save!")
                    self.Counter = 0
                    self.VectorData = []
                    self.VectorData1 = []
                    self.VectorData2 = []
                print("DATA Counter:", self.Counter)
        if self.ResetCounterCap and self.statusObject == False:
            self.Counter = 0
            self.VectorData = []
            self.VectorData1 = []
            self.VectorData2 = []

    def prediction_mode(self) -> None:
        pass


def main():
    os.system('clear')
    time.sleep(1)
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
    time.sleep(1)
    op = args.mode
    if op == 1:
        print("Trainging mode")
    else:
        print("Prediction mode")
    while not rospy.is_shutdown():
        if op == 1:
            objNode.save_data_set()
        else:
            objNode.prediction_mode()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
