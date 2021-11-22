"""
By Kevin Medrano Ayala
Descripcion: Modulo para deteccion de objetos con Yolov3
"""
# Importar modulos
import cv2 as cv
import numpy as np

# Parametros iniciales
cap = cv.VideoCapture(0)
whT = 320
confThreshold =0.5
nmsThreshold= 0.2
classNames = []
classesFile = "yolo-coco/coco.names"
modelConfiguration = "yolo-coco/yolov3.cfg"
modelWeights = "yolo-coco/yolov3.weights"

# Cargar modelos y pesos
classNames = open(classesFile).read().strip().split("\n")
#print(classNames)
net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
# Especificar uso de CPU 
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

# Metodo encontrar objetos
def findObjects(outputs,img):
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []
    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w,h = int(det[2]*wT) , int(det[3]*hT)
                x,y = int((det[0]*wT)-w/2) , int((det[1]*hT)-h/2)
                bbox.append([x,y,w,h])
                classIds.append(classId)
                confs.append(float(confidence))
    indices = cv.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)

    for i in indices:
        i = i
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]
        cv.rectangle(img, (x, y), (x+w,y+h), (255, 0 , 255), 2)
        cv.putText(img,f'{classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                  (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
print("Running...")
while True:
    success, img = cap.read()
    blob = cv.dnn.blobFromImage(img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
    net.setInput(blob)
    layersNames = net.getLayerNames()
    outputNames = [(layersNames[i - 1]) for i in net.getUnconnectedOutLayers()]
    outputs = net.forward(outputNames)
    findObjects(outputs,img)
    cv.imshow('Image', img)
    cv.waitKey(1)