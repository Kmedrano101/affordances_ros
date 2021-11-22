"""
By Kevin Medrano Ayala
Descripcion: Modulo para deteccion de poses del cuerpo humano
"""
# Importar modulos
import cv2
import mediapipe as mp
import time
import math

# Clase detector de pose
class poseDetector():
    def __init__(self):
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
    
def main():
    cap = cv2.VideoCapture(0)
    detector = poseDetector()
    print("Running...")
    while True:
        success, img = cap.read()
        img = detector.findPose(img)
        lmList = detector.findPosition(img, draw=False)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
if __name__ == "__main__":
    main()