#!/usr/bin/env python3

# Importar modulos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
import mediapipe as mp
import time
from os import system
import cv2 as cv

# Parametros y variables
try:
	rospy.get_param_names()
except ROSException:
	print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()  
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_pp_name",default="person_pose")
TOPIC_S1_NAME = rospy.get_param(PACKAGE_NAME+"subscribers/person_object_flag/topic", default="/affordances/flag")
TOPIC_P1_NAME = rospy.get_param(PACKAGE_NAME+"publishers/status_person_pose/topic",default="/affordances/status_person_pose")

# Clase 
class PersonPose:
	"""Person object"""
	def __init__(self):
		self._objectPos = None					# Only Private no external access
		self._cvFrame = []						# Only Private no external access
		self._subTopicBorderBoxesName = None 	# Private and external access 
		self._subTopicImageName = None 			# Private and external access
		self._pubTopicStatusNodeName = None		# Private and external access
		self._pubTopicStatusNode = None			# Private and external access
		self._statusNode = False				# Private and external access
		self._borderBoxes = ()					# Only Private no external access
		self._dataReceivedTopic1 = False		# Private and external access
		self._dataReceivedTopic2 = False		# Private and external access
		self._dataPosReady = False				# Only Private no external access
		self._posObject = ()					# Only Private no external access
		self.mpDraw = mp.solutions.drawing_utils
		self.mpPose = mp.solutions.pose
		self.pose = self.mpPose.Pose()
    
	def findPose(self, img, draw=True):
		imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.results = self.pose.process(imgRGB)
		if self.results.pose_landmarks:
			if draw:
				self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
		return img
    
	def findPosition(self, img, draw=True):
		self.lmList = []
		if self.results.pose_landmarks:
			for id, lm in enumerate(self.results.pose_landmarks.landmark):
				h, w, c = img.shape
				cx, cy = int(lm.x * w), int(lm.y * h)
				self.lmList.append([id, lm.x, lm.y])
				if draw:
					cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
		return self.lmList
	"""Propierties"""
	

	""" Methods and Actions"""
		
	def start_subscribers(self) -> None:
		rospy.Subscriber(self._subTopicImageName, Image, self.image_source_callback)
		rospy.Subscriber(self._subTopicBorderBoxesName, BoundingBoxes, self.border_boxes_callback)

	def start_publishers(self) -> None:
		self._pubTopicStatusNode = rospy.Publisher(self._pubTopicStatusNodeName, Bool, queue_size=10)

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
	objNode = PersonPose()
	objNode.subTopicBorderBoxesName = TOPIC_S1_NAME
	objNode.pubTopicStatusNodeName = TOPIC_P1_NAME
	objNode.start_subscribers()
	objNode.start_publishers()
	time.sleep(2)
	while not rospy.is_shutdown():
		if objNode._dataReceivedTopic1 == False and objNode._dataReceivedTopic2 == False: #or objNode._dataReceivedTopic2 == False:
			print("[WARNING] Datos no recibidos")
		else:
			objNode.get_pos()
			objNode.show_img()
		#rate.sleep()
	rospy.spin()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass