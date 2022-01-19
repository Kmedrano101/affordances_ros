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
        self.aTime = time.time()
        self.sTime = 0.1
        self.iTime = 0
        self.fValue = 0
        self.X1 = 0
        self.Y1 = 0
        self.LVel = []

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
                self.lmList.append([id, lm.x, lm.y,lm.visibility])
                if id == 0:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 0), cv2.FILLED)
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        return self.lmList
    
    def findVelocity(self,img):
        h, w, c = img.shape
        if self.results.pose_landmarks:
            lm = self.results.pose_landmarks.landmark[0]
            if self.fValue == 0:
                self.X1, self.Y1 = int(lm.x * w), int(lm.y * h)
                self.fValue = 1
            self.aTime = time.time()
            if self.aTime > (self.sTime+self.iTime):
                X2, Y2 = int(lm.x * w), int(lm.y * h)
                #print(f"X2: {X2} Y2: {Y2}")
                V = abs(self.X1 - X2) + abs(self.Y1 - Y2)
                self.LVel.append(V)
                if len(self.LVel) >= 5:
                    Vreal = sum(self.LVel)/len(self.LVel)
                    print("Velocity: ",int(Vreal))
                    self.LVel = []
                self.fValue = 0
                self.iTime = time.time()
                
            
def main():
    cap = cv2.VideoCapture(0)
    detector = poseDetector()
    print("Running...")
    pTime = 0
    while True:
        success, img = cap.read()
        img = detector.findPose(img)
        lmList = detector.findPosition(img, draw=False)
        detector.findVelocity(img)
        contador = 0
        for num,x in enumerate(lmList):
            if num == 0 or num == 15 or num == 16:
                if x[3]>0.5:
                    contador += 1
        if contador >= 3:
            print("Listo para captura de datos!!!! eres un crack")
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN,
                3, (255, 0, 0), 3)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
if __name__ == "__main__":
    main()