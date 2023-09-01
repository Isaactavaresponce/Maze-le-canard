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
        elif (vic=="amarillo"):
            GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
            
            GPIO.output(rojo, GPIO.HIGH)#PRENDE Y APAGA 2 VECES EL LED ROJO INDICANDO LOS BOTIQUINES
            time.sleep(5)
            GPIO.output(rojo, GPIO.LOW)
            time.sleep(2)
            GPIO.output(rojo, GPIO.HIGH)
            time.sleep(5)
            GPIO.output(rojo, GPIO.LOW)
            time.sleep(2)
            botiquines=botiquines-2
            
            
            GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
            
            color = '2' #Salimos de la victima
            color=color +"\n"
            arduino(color)
            time.sleep(0.1)
            color = '0' #Salimos de la victima
            color=color +"\n"
            arduino(color)    
    else :
        GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
        time.sleep(2)
        GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
        color = '2' #Salimos de la victima
        color=color +"\n"
        arduino(color)
        time.sleep(0.1)
        color = '0' #AVANZA de la victima
        color=color +"\n"
        arduino(color) 

                           
    
    
victims = []
a = 0



def letra(let):
    if let <=4:
        cap.release()
            
        print("detenemos la deteccion")
        
        color = "1"
        color=color +"\n"
        arduino(color)  
        vic="rojo"
        print(vic)  
        detecta(vic)

        #print(color)
        cap = cv2.VideoCapture(-1) 
        print("reinicioamos la deteccion")

    elif let<=9:
        cap.release()
            
        print("detenemos la deteccion")
        
        color = "1"
        color=color +"\n"
        arduino(color)  
        vic="amarillo"
        print(vic)  
        detecta(vic)

        #print(color)
        cap = cv2.VideoCapture(-1) 
        print("reinicioamos la deteccion")

def ord_pt(puntos):
    n_puntos = np.concatenate([puntos[0], puntos[1], puntos[2], puntos[3]]).tolist()
    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])
    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])
    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])
    
    return [x1_order[0], x1_order[1], x2_order[0], x2_order[1]]





print("iniciando colores")

while True:
    _, img = cap.read()
    
    vic=""

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ### red color ###

    red_lower = np.array([136,87,111],np.uint8)
    red_upper = np.array([180,255,255],np.uint8)

    ### yellow color ###

    yellow_lower = np.array([15,100,20],np.uint8)
    yellow_upper = np.array([45,255,255],np.uint8)

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
    contours, hierarchy = cv2.findContours(yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, "AMARILLO: ", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            
            cap.release()
            
            print("detenemos la deteccion")
            
            color = "1"
            color=color +"\n"
            arduino(color)  
            vic="amarillo"
            print(vic)  
            detecta(vic)

            #print(color)
            cap = cv2.VideoCapture(-1) 
            print("reinicioamos la deteccion")
            
            
    orig = img.copy()
    
    canny = cv2.Canny(img,0,255)
    canny = cv2.dilate(canny,None,iterations=2)


    cnts,_ = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        area = cv2.contourArea(c)
        x,y,w,h = cv2.boundingRect(c)
        epsilon = 0.09*cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,epsilon,True)
        aspect_ratio = float(w/h)

        if len(approx)==4 and aspect_ratio>0.7 and aspect_ratio<1.2 and area > 7000 and area < 90000:
            cv2.drawContours(img, [approx], 0, (0,255,255),2)

            puntos = ord_pt(approx)
            pts1 = np.float32(puntos)
            pts2 = np.float32([[0,0], [w,0], [0,h], [w,h]])
            m = cv2.getPerspectiveTransform(pts1,pts2)
            giro = cv2.warpPerspective(orig, m, (w,h))
            girogr = cv2.cvtColor(giro, cv2.COLOR_BGR2GRAY)
            cv2.imshow("giro",giro)

            for v in victims:
                v = cv2.resize(v, (girogr.shape[1], girogr.shape[0]))
                res = cv2.matchTemplate(girogr, v, cv2.TM_CCOEFF_NORMED)
                min, max, pos_min, pos_max = cv2.minMaxLoc(res)
                print(max,"       ",min)
                #print("imagen ", a)
                maxde = max*100
                print(maxde)
                if maxde > 50:
                    letra(a)
                a = a + 1
            a = 0       

    #cv2.imshow("Color Tracking", img)
    if cv2.waitKey(10) & 0xFF == ord('x' or 'X'):
        break


ard.close()
ard.flushInput()
cap.release()
cv2.destroyAllWindows()