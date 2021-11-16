#!/usr/bin/env python3
# Librerias
import mediapipe as mp
import cv2
import time
import numpy as np
import pandas as pd
import os
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from xgboost import XGBClassifier


# Variables
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils  
points = mpPose.PoseLandmark  
path = "DATASET/TRAIN/warrior2" 
data = []
cap = cv2.VideoCapture(0)
mpdraw = mp.solutions.drawing_utils

"""
# Creando frame
for p in points:
        x = str(p)[13:]
        data.append(x + "_x")
        data.append(x + "_y")
        data.append(x + "_z")
        data.append(x + "_vis")
data = pd.DataFrame(columns = data) # Empty dataset
# print(data, data.shape) #(0,132)

count = 0
for img in os.listdir(path):
        temp = []
        img = cv2.imread(path + "/" + img)
        imageWidth, imageHeight = img.shape[:2]
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        blackie = np.zeros(img.shape) # Blank image
        results = pose.process(imgRGB)

        if results.pose_landmarks:
                # mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS) #draw landmarks on image
                # draw landmarks on blackie
                mpDraw.draw_landmarks(
                    blackie, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
                landmarks = results.pose_landmarks.landmark
                for i,j in zip(points,landmarks):
                        temp = temp + [j.x, j.y, j.z, j.visibility]
                data.loc[count] = temp
                count +=1

        cv2.imshow("Image", img)
        cv2.imshow("blackie",blackie)
        cv2.waitKey(100)

data.to_csv("ds_warrior2.csv") # save the data as a csv file

data1 = pd.read_csv("ds_plank.csv")
data1['y'] = np.add(0,1)
data2 = pd.read_csv("ds_warrior2.csv")
data2['y'] = np.add(0,2)
data3 = pd.read_csv("ds_goddess.csv")
data3['y'] = np.add(0,3)
data4 = pd.read_csv("ds_tree.csv")
data4['y'] = np.add(0,4)
data5 = pd.read_csv("ds_downdog.csv")
data5['y'] = np.add(0,5)
"""
data = pd.read_csv("ds.csv")
data_final = data.sample(frac=1).reset_index(drop=True)

X, y = data.iloc[:, :132].values, data['y'].values
#print(X)
#print(y)

# Dividir el data set en conjunto de entrenamiento y conjunto de testing
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.20, random_state = 0)

model1 = SVC(kernel='linear', decision_function_shape='ovo')
model1.fit(X_train, y_train)

model2 = DecisionTreeClassifier(
           max_depth         = 5,
           criterion         = 'entropy',
           random_state      = 1
          )
model2.fit(X_train,y_train)

model3 = XGBClassifier(random_state = 11)
model3.fit(X_train, y_train)

# Predicción de los resultados con el Conjunto de Testing
y_pred1 = model1.predict(X_test)
y_pred2  = model2.predict(X_test)
y_pred3  = model3.predict(X_test)

# Elaborar una matriz de confusión
from sklearn.metrics import confusion_matrix
from sklearn import metrics

#cm = confusion_matrix(y_test, y_pred1)

print("SVM Accuracy: ",metrics.accuracy_score(y_test,y_pred1))
print("RF Accuracy: ",metrics.accuracy_score(y_test,y_pred2))
print("ADA Accuracy: ",metrics.accuracy_score(y_test,y_pred3))

y = 0
temp = []
#cad = input("Path of image:")
path = "DATASET/TEST/plank/00000006.jpg"
img = cv2.imread(path)
imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
results = pose.process(imgRGB)
asan = "Nothing"

if results.pose_landmarks:
        mpDraw.draw_landmarks(img, results.pose_landmarks,mpPose.POSE_CONNECTIONS)
        landmarks = results.pose_landmarks.landmark
        for j in landmarks:
                temp = temp + [j.x, j.y, j.z, j.visibility]
        print("************")
        print("Model 1 result: ")
        y = model1.predict([temp])
        if y==1:
                print("plank")
                asan = "plank"
        if y==2:
                print("warrior")
                asan = "warrior"
        if y==3:
                print("goddess")
                asan = "goddess"
        if y==4:
                print("tree")
                asan = "tree"
        if y==5:
                print("downdog")
                asan = "downdog"
        print("************")
        print("Model 2 result: ")
        y = model2.predict([temp])
        if y==1:
                print("plank")
        if y==2:
                print("warrior")
        if y==3:
                print("goddess")
        if y==4:
                print("tree")
        if y==5:
                print("downdog")
        print("************")
        print("Model 3 result: ")
        y = model3.predict([temp])
        if y==1:
                print("plank")
        if y==2:
                print("warrior")
        if y==3:
                print("goddess")
        if y==4:
                print("tree")
        if y==5:
                print("downdog")
        cv2.putText(img, asan, (50, 50),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 3)
        cv2.imshow("image", img)
        cv2.imwrite("img_mod.jpg", img)
        cv2.waitKey(0)

condicion = True
while condicion!=True:
    success,img = cap.read()
    imgRGB  = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
        for j in landmarks:
                temp = temp + [j.x, j.y, j.z, j.visibility]
        try:
                y = model1.predict([temp])
        except Exception as e:
                pass        
        if y==1:
                print("plank")
                asan = "plank"
        if y==2:
                print("warrior")
                asan = "warrior"
        if y==3:
                print("goddess")
                asan = "goddess"
        if y==4:
                print("tree")
                asan = "tree"
        if y==5:
                print("downdog")
                asan = "downdog"
        print(asan)
        cv2.putText(img, asan, (50,50), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),3)

    cv2.imshow("image",img)
    cv2.waitKey(1)

