import mediapipe as mp
import cv2
import time
import numpy as np
import os
"""mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils 
points = mpPose.PoseLandmark 
print(os.path.abspath ('.'))
path = "/home/kmedrano101/catkin_ws/src/affordances/data/DATASET/TRAIN/plank"
data = []
for p in points:
    x = str(p)[13:]
    data.append(x + "_x")
    data.append(x + "_y")
    data.append(x + "_z")
    data.append(x + "_vis")
data = pd.DataFrame(columns=data) 
count = 0

for img in os.listdir(path):
    temp = []
    img = cv2.imread(path + "/" + img)
    imageWidth, imageHeight = img.shape[:2]
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    blackie = np.zeros(img.shape)
    results = pose.process(imgRGB)
    if results.pose_landmarks:
        mpDraw.draw_landmarks(
            blackie, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
        landmarks = results.pose_landmarks.landmark
        for i, j in zip(points, landmarks):
            temp = temp + [j.x, j.y, j.z, j.visibility]
        data.loc[count] = temp
        count += 1
    cv2.imshow("Image", img)
    cv2.imshow("blackie", blackie)
    cv2.waitKey(100)
data.to_csv("data_test.csv")  

from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
data = np.array([344, 31,  4])
#scaler.transform(data)
data = scaler.fit_transform(data[:,np.newaxis]).tolist()
data = [data[0][0],data[1][0],data[2][0]]
print(data)"""
import csv
listac = [0.5597094297409058, 0.5346274971961975, -0.6361134648323059, 0.999983012676239, 0.7518019080162048, 0.8171274065971375, -0.18145349621772766, 0.9994426965713501, 0.36200764775276184, 0.8129791021347046, -0.05294239521026611, 0.9997353553771973, 0.9138382077217102, 1.1692306995391846, -0.35242852568626404, 0.4192102551460266, 0.1977972388267517, 1.159598469734192, -0.49076586961746216, 0.885757565498352, 0.9487541913986206, 1.331005573272705, -0.7456013560295105, 0.5162373781204224, 0.20965774357318878, 0.9314325451850891, -1.4411869049072266, 0.9853494763374329, 0.6638270020484924, 1.513633370399475, -0.09722878038883209, 0.0005769540439359844, 0.3988609313964844, 1.5026071071624756, 0.10225944221019745, 0.0007838639430701733, 0.6400516033172607, 2.1010830402374268, -0.17798125743865967, 0.0007889706757850945, 0.39523643255233765, 2.099994421005249, 0.2658478021621704, 0.0003409872006159276, 0.6342825889587402, 2.6312108039855957, 0.25065577030181885, 7.668346370337531e-05, 0.3928215801715851, 2.6136014461517334, 0.6992189288139343, 3.556310730346013e-06]
if listac:
    print("Datos")
    with open('dataset.csv', 'a') as f:
        write = csv.writer(f,delimiter=':', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        write.writerow(listac)
    with open("dataset.txt", "a") as file:
        file.write("\n"+str(listac))
    #time.sleep(5)
else:
    print("Datos NO")

#####################################################################
# Determinar posiciones del objeto en base a puntos claves del cuerpo
# Topic para lanzar informacion de acciones que determina el robot
