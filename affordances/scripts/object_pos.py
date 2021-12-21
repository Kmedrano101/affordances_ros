#!/usr/bin/env python3

# Importar modulos
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from std_msgs.msg import Bool
import time
import threading
from os import system
import cv2 as cv

# Parametros y variables
try:
	rospy.get_param_names()
except ROSException:
	print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = rospy.get_namespace()  
NODE_NAME = rospy.get_param(PACKAGE_NAME+"node_op_name",default="object_pos")
TOPIC_S1_NAME = rospy.get_param(PACKAGE_NAME+"subscribers/border_boxes/topic", default="/affordances/object_bounding_boxes")
TOPIC_S2_NAME = rospy.get_param(PACKAGE_NAME+"subscribers/image_source/topic", default="/usb_cam/image_raw")
TOPIC_P1_NAME = rospy.get_param(PACKAGE_NAME+"publishers/status_object_pos/topic",default="/affordances/status_object_pos")
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv.TrackerCSRT_create,
	"kcf": cv.TrackerKCF_create,
	"mil": cv.TrackerMIL_create,
	}
TRACKER = OPENCV_OBJECT_TRACKERS["csrt"]() # Test these objects tracker
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
		self._pubTopicStatusNode = None			# Private and external access
		self._statusNode = False				# Private and external access
		self._borderBoxes = ()					# Only Private no external access
		self._dataReceivedTopic1 = False		# Private and external access
		self._dataReceivedTopic2 = False		# Private and external access
		self._dataPosReady = False				# Only Private no external access
		self._posObject = ()					# Only Private no external access

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

	""" Methods and Actions"""
	def image_source_callback(self,msg) -> None:
		self._dataReceivedTopic1 = True
		try:
			self._cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(f"[ERROR] {e}")	

	def border_boxes_callback(self,msg) -> None:
		self._dataReceivedTopic2 = True
		self._borderBoxes = msg.bounding_boxes
		
	def start_subscribers(self) -> None:
		rospy.Subscriber(self._subTopicImageName, Image, self.image_source_callback)
		rospy.Subscriber(self._subTopicBorderBoxesName, BoundingBoxes, self.border_boxes_callback)

	def start_publishers(self) -> None:
		self._pubTopicStatusNode = rospy.Publisher(self._pubTopicStatusNodeName, Bool, queue_size=10)

	def get_pos(self) -> None:
		pos = []
		""" Funcion para obtener la posicion central del objeto a realizar el tracking"""
		if not self._dataPosReady and self._dataReceivedTopic2:
			for box in self._borderBoxes:
				if box.Class == "cup":	# Considerar varios objetos del mismo tipo IMPORTANT
					rospy.loginfo("Object CUP encontrado.")
					#print(f"Data ({box.xmin},{box.ymin}, {box.xmax}, {box.ymax})")
					pos.append(box.xmin-2)
					pos.append(box.ymin-1)
					pos.append(abs(box.ymax - box.ymin))
					pos.append(abs(box.xmax - box.xmin)+5)
					self._dataPosReady = True
					self._posObject = tuple(pos)
					break
	
	def show_img(self) -> None:
		if self._dataPosReady and self._dataReceivedTopic1: # Verificar y validar datos recibidos enfocar seguimiento de objeto
			#print(f"Datos desde show_img {self.get_pos()}")
			if not self._statusNode:
				TRACKER.init(self._cvFrame,self._posObject)
				rospy.loginfo("Tracker started.")
				#threading.Timer(4, self.restart_capture_data).start()
			self._statusNode = True
			(success, box) = TRACKER.update(self._cvFrame)
			if success:
				(x, y, w, h) = [int(v) for v in box]
				cv.rectangle(self._cvFrame, (x, y), (x + h, y + w),(0, 255, 0), 2)
				cv.circle(self._cvFrame,(int((x*2+h)/2),int((y*2+w)/2)),4,(255,0,0),-1)
			cv.imshow("Image from Node ObjectPos", self._cvFrame)
			cv.waitKey(1)
	def restart_capture_data(self) -> None:
		self._dataPosReady = False
		self._statusNode = False
		TRACKER = OPENCV_OBJECT_TRACKERS["csrt"]()
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
	objNode = ObjectPos()
	objNode.subTopicBorderBoxesName = TOPIC_S1_NAME
	objNode.subTopicImageName = TOPIC_S2_NAME
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
