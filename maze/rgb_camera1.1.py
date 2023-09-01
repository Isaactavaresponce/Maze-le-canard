import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO #libreria de los los pines de la rasp

cap = cv2.VideoCapture(-1) 


ard = serial.Serial("/dev/ttyACM0", baudrate=9600)
def arduino(comando):
    x = comando
    comandoBytes = x.encode()
    ard.write(comandoBytes)
    time.sleep(0.1)
    read = ard.readline()
    print(read)
    
    
    
    
verde = 16
rojo=12


vic=""
GPIO.setwarnings(False)
# Configura el modo de los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(verde, GPIO.OUT)
GPIO.setup(rojo, GPIO.OUT)
botiquines = 12

def detecta(vic):
    global botiquines
    if botiquines !=0 :
        if(vic=="rojo"):
            GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
            
            GPIO.output(rojo, GPIO.HIGH)#PRENDE Y APAGA 3 VECES EL LED ROJO INDICANDO LOS BOTIQUINES
            time.sleep(5)
            GPIO.output(rojo, GPIO.LOW)
            time.sleep(2)
            GPIO.output(rojo, GPIO.HIGH)
            time.sleep(5)
            GPIO.output(rojo, GPIO.LOW)
            time.sleep(2)
            GPIO.output(rojo, GPIO.HIGH)
            time.sleep(5)
            GPIO.output(rojo, GPIO.LOW)
            time.sleep(2)
            
            GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
            
            color = '2' #Salimos de la victima
            color=color +"\n"
            arduino(color)
            time.sleep(0.1)
            color = '0' #Salimos de la victima
            color=color +"\n"
            arduino(color)
            botiquines=botiquines-3
           
    else :
        GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
        time.sleep(2)
        GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
        color = '2' #Salimos de la victima
        color=color +"\n"
        arduino(color)
        time.sleep(0.1)
        color = '0' #Salimos de la victima
        color=color +"\n"
        arduino(color) 

                           
    
    




print("iniciando colores")

while True:
    _, img = cap.read()
    
    vic=""

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ### red color ###

    red_lower = np.array([0,170,150],np.uint8)
    red_upper = np.array([10,200,200],np.uint8)

    ### yellow color ###

    yellow_lower = np.array([15,160,130],np.uint8)
    yellow_upper = np.array([45,170,200],np.uint8)

        # all color together
    red = cv2.inRange(hsv, red_lower, red_upper)
    yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Morphological Transform, Dilation

    kernal = np.ones((5, 5), "uint8")

    red = cv2.dilate(red, kernal)
    res_red = cv2.bitwise_and(img, img, mask = red)

    yellow = cv2.dilate(yellow, kernal)
    res_yellow = cv2.bitwise_and(img, img, mask = yellow)

    # Tracking red
    contours, hierarchy = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "ROJO: ", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
            
            print("detenemos la deteccion")
            cap.release()
            
                        
            color = '1'
            color=color +"\n"
            arduino(color)
            vic="rojo"
            print(vic)

            detecta(vic)
            
            cap = cv2.VideoCapture(-1) 
            print("reinicioamos la deteccion")
            
            #print(color + ' rasp')
            

    # Tracking yellow
    
            

    #cv2.imshow("Color Tracking", img)
    if cv2.waitKey(10) & 0xFF == ord('x' or 'X'):
        break


ard.close()
ard.flushInput()
cap.release()
cv2.destroyAllWindows()