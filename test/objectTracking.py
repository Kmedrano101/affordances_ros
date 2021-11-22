"""
By Kevin Medrano Ayala
Descripcion: Seguidor de posicion para objetos
"""
# Importar modulos
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import cv2 as cv

# Parametros iniciales
cap = cv.VideoCapture(0)
# Definir tipo de tracker
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv.TrackerCSRT_create,
	"kcf": cv.TrackerKCF_create,
	"mil": cv.TrackerMIL_create,
	}
tracker = OPENCV_OBJECT_TRACKERS["csrt"]()
initBB = None

# Bucle principal
print("Running...")
while True:
	rec, img = cap.read()
	# Redefinir size img
	frame = imutils.resize(img, width=500)
	(H, W) = frame.shape[:2]
	if initBB is not None:
		(success, box) = tracker.update(frame)
		if success:
			(x, y, w, h) = [int(v) for v in box]
			cv.rectangle(frame, (x, y), (x + w, y + h),
				(0, 255, 0), 2)
		fps.update()
		fps.stop()
		info = [
			("Tracker", "CSRT"),
			("Success", "Yes" if success else "No"),
			("FPS", "{:.2f}".format(fps.fps())),
		]
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv.putText(frame, text, (10, H - ((i * 20) + 20)),
				cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
	cv.imshow("Frame", frame)
	key = cv.waitKey(1) & 0xFF
	# Obtener frame para seguir 
	if key == ord("s"):
		initBB = cv.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
		tracker.init(frame, initBB)
		fps = FPS().start()