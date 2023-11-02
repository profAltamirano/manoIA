import cv2
import mediapipe as mp
import numpy as np
from math import acos, degrees
import serial
import time
ser = serial.Serial('COM8',9600,timeout=1)
time.sleep(2)
#define el centroide de la palma
def palm_centroid(coordinates_list):
    coordinates = np.array(coordinates_list)
    centroid = np.mean(coordinates, axis=0)
    centroid = int (centroid[0]), int(centroid[1])
    return centroid

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#PULGAR son los puntos del pulgar
thumb_points = [1, 2, 4]
#indice,medio,anular,son los puntos del sistema
palm_points = [0,1,2,5,9,13,17]
fingertips_points = [8,12,16,20]
finger_base_points = [6,10,14,18]

##colores para los recuadros de los dedos
GREEN = (48,255,48)
BLUE = (192,101,21)
YELLOW = (0,204,255)
PURPLE = (128,64,128)
PEACH = (180,229,255)

with mp_hands.Hands(
    model_complexity=1,
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    while True:
        ret, frame = cap.read()
        if ret == False:
            break
        
        height, width, _ = frame.shape
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)
        thickness = [2,2,2,2,2]
        dato = ['a','b','c','d','e']
        if results.multi_hand_landmarks:
            coordinates_thumb = []
            coordinates_palm = []
            coordinates_ft = []
            coordinates_fb = []
            for hand_landmarks in results.multi_hand_landmarks:
                for index in thumb_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_thumb.append([x,y])          

                for index in palm_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_palm.append([x,y])
                
                for index in fingertips_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_ft.append([x,y])

                for index in finger_base_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_fb.append([x,y])
                #pulgar el array  para determinar los puntos del sistema
                p1 = np.array(coordinates_thumb[0])
                p2 = np.array(coordinates_thumb[1])
                p3 = np.array(coordinates_thumb[2])

                l1 = np.linalg.norm(p2 - p3)
                l2 = np.linalg.norm(p1 - p3)
                l3 = np.linalg.norm(p1 - p2)

                #calcular angulo
                angle = degrees(acos((l1**2 + l3**2 - l2**2)/(2*l1*l3)))
                #print(angle)
                thumb_finger = np.array(False)
                if angle>150:
                    thumb_finger = np.array(True)
                #print("thumb finger: ", thumb_finger)



                #indice medio anular meñique el dedo indice de 
                nx, ny = palm_centroid(coordinates_palm)
                cv2.circle(frame, (nx, ny), 3, (0, 255, 0), 2)

                coordinates_centroid = np.array([nx, ny])
                coordinates_ft = np.array(coordinates_ft)
                coordinates_fb = np.array(coordinates_fb)

                #distancias ft determina la punta de los dedos fb determina la base de los dedos
                d_centroid_ft = np.linalg.norm(coordinates_centroid - coordinates_ft, axis=1)
                d_centroid_fb = np.linalg.norm(coordinates_centroid - coordinates_fb, axis=1)

                dif = d_centroid_ft - d_centroid_fb

                fingers = dif > 0
                fingers = np.append(thumb_finger, fingers)
                #print("dedos: ",fingers)muestra un array con el valor true o folse de cada dedo
                #print("dedos: ",fingers)
                ## 1
                if fingers[0] == False:
                    ser.write(b'a')
                    time.sleep(0.03)
                else:
                    ser.write(b'b')
                    time.sleep(0.03)
                ### 2 dedo anular 
                if fingers[1] == False:
                    ser.write(b'c')
                    time.sleep(0.03)
                else:
                    ser.write(b'd')
                    time.sleep(0.03)
                ### 3
                if fingers[2] == False:
                    ser.write(b'e')
                    time.sleep(0.03)
                else:
                    ser.write(b'f')
                    time.sleep(0.03)
                ### 4
                if fingers[3] == False:
                    ser.write(b'g')
                    time.sleep(0.03)
                else:
                    ser.write(b'h')
                    time.sleep(0.03)
                ###
                if fingers[4] == False:
                    ser.write(b'i')
                    time.sleep(0.03)
                else:
                    ser.write(b'j')
                    time.sleep(0.03)



                for(i, finger) in enumerate(fingers):
                    if finger == True:
                        thickness[i] = -1

    
              

                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        ####visualizacion###
        #pulgar

        
        cv2.rectangle(frame, (100,10), (150,60), PEACH, thickness[0])
        cv2.putText(frame, "PULGAR", (100,80),1,1,(255,255,255),2 )
        #indice
        cv2.rectangle(frame, (160,10), (210,60), PURPLE, thickness[1])
        cv2.putText(frame, "INDICE", (160,80),1,1,(255,255,255),2 )
        #medio
        cv2.rectangle(frame, (220,10), (270,60), YELLOW, thickness[2])
        cv2.putText(frame, "MEDIO", (220,80),1,1,(255,255,255),2)
        #anular
        cv2.rectangle(frame, (280,10), (330,60), GREEN, thickness[3])
        cv2.putText(frame, "ANULAR", (280,80),1,1,(255,255,255),2 )
        #MEÑIQUE
        cv2.rectangle(frame, (340,10), (390,60), BLUE, thickness[4])
        cv2.putText(frame, "MENIQUE", (340,80),1,1,(255,255,255),2 )

        cv2.imshow('Frame',frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
cap.release()
cv2.destroyAllWindows()
ser.close()