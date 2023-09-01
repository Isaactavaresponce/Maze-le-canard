import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO #libreria de los los pines de la rasp

cap = cv2.VideoCapture(-1)

victims = []
a = 0

ard = serial.Serial("/dev/ttyACM0", baudrate=9600)

verde = 16
rojo=12

vic=""

GPIO.setwarnings(False)

# Configura el modo de los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(verde, GPIO.OUT)
GPIO.setup(rojo, GPIO.OUT)

botiquines = 12

def ord_pt(puntos):
    n_puntos = np.concatenate([puntos[0], puntos[1], puntos[2], puntos[3]]).tolist()
    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])
    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])
    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])
    
    return [x1_order[0], x1_order[1], x2_order[0], x2_order[1]]

def arduino(comando):
    x = comando
    comandoBytes = x.encode()
    ard.write(comandoBytes)
    time.sleep(0.1)
    read = ard.readline()
    print(read)
    

def detecta(vic):
    if(vic=="rojo"):
        GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
        
        GPIO.output(rojo, GPIO.HIGH)#PRENDE Y APAGA 3 VECES EL LED ROJO INDICANDO LOS BOTIQUINES
        
        
        GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
        
        
        time.sleep(0.1)
        color = '0\n' #Salimos de la victima
        
        arduino(color)
        
    elif(vic=="amarillo"):
        GPIO.output(verde, GPIO.HIGH)#prende para demostrar que encontro victima
        
        GPIO.output(rojo, GPIO.HIGH)#PRENDE Y APAGA 3 VECES EL LED ROJO INDICANDO LOS BOTIQUINES
        
        
        GPIO.output(verde, GPIO.LOW)#apagamos para demostrar que salimos de la victima victima
        
        
        time.sleep(0.1)
        color = '0\n' #Salimos de la victima
        
        arduino(color)
        

    


def letra(let):
    global cap,vic,a
    if let <=4:
        wor="H"
        cap.release()
            
        print("detenemos la deteccion  por h")
        vic="rojo"

        detecta(vic)
        vic=""
        time.sleep(1000)
        
        cap = cv2.VideoCapture(-1) 
        print("reinicioamos la deteccion")

    elif let<=9:
        wor="S"
        cap.release()
            
        print("detenemos la deteccion por s")
        vic="amarillo"

        detecta(vic)
        vic=""
        time.sleep(1000)
        
        cap = cv2.VideoCapture(-1) 
        print("reinicioamos la deteccion")


for i in range(10):
    ej = cv2.imread(r'/home/pi/camara_maze/images/vic_0'+str(i)+'.jpg', 0)
    victims.append(ej)


def compara():
    global cap,vic,a
    _, img = cap.read()
    orig = img.copy()
    vic=""
    canny = cv2.Canny(img,0,255)
    canny = cv2.dilate(canny,None,iterations=4)


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
            #cv2.imshow("giro",giro)

            for v in victims:
                v = cv2.resize(v, (girogr.shape[1], girogr.shape[0]))
                res = cv2.matchTemplate(girogr, v, cv2.TM_CCOEFF_NORMED)
                min, max, pos_min, pos_max = cv2.minMaxLoc(res)
                #print(max,"       ",min)
                #print("imagen ", a)
                maxde = max*100
                print(maxde)
                if maxde > 30:
                    letra(a)
                a = a + 1
            a = 0

    #cv2.imshow("Grabando",img)
    #cv2.imshow("canny",canny)


def det_color():
    global cap ,vic
    _, img = cap.read()
    
    vic=""

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ### red color ###

    red_lower = np.array([0,150,150],np.uint8)
    red_upper = np.array([10,200,200],np.uint8)

    ### yellow color ###

    yellow_lower = np.array([15,160,30],np.uint8)
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
        if(area > 500):
            x, y, w, h = cv2.boundingRect(contour)            
            cap.release()
            
            print("detenemos la deteccion por rojo color")
            
            color = "1\n"
            
            arduino(color)  
            vic="rojo"
            print(vic)  
            detecta(vic)

            #print(color)
            cap = cv2.VideoCapture(-1) 
            print("reinicioamos la deteccion")
            #print(color + ' rasp')
            

    # Tracking yellow
    contours, hierarchy = cv2.findContours(yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            
            cap.release()
            
            print("detenemos la deteccion por amarillo")
            
            color = "1\n"
            
            arduino(color)  
            vic="amarillo"
            print(vic)  
            detecta(vic)

            #print(color)
            cap = cv2.VideoCapture(-1) 
            print("reinicioamos la deteccion")
            

    #cv2.imshow("Color Tracking", img)
time.sleep(2)

while True:
    vic=""
    det_color()
    #time.sleep(0.001)
    #compara()

  

ard.close()
ard.flushInput()
cap.release()
cv2.destroyAllWindows()