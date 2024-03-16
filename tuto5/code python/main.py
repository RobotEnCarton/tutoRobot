#!/usr/bin/env python3

from gpiozero import Button


import serial
import time
import sys
import threading
from rplidar import RPLidar
import numpy as np
import cv2
from math import *
import printScore

posX = 0.0
posY = 0.0
posZ = 0.0
start_time = 0
mainScore = 0

runningMatch = False
debug = False
debugCV = True
lastV = "0"
cote = ""

targX = 150.0
targY = 100.0
# parametre de trajectoire
a = 0
b = 0
c = 0
dist = 0

affCoef = 2  # echelle d'affichage
affSize = 20  # taille de la pastille
# ~ offsetAngle = 274  # rectification d'angle en °
offsetAngle = 90  # rectification d'angle en °
minX = 0  # en cm
minY = 0  # en cm
alertDist = 50  # en cm

def print_there(x, y, text):
    sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
    sys.stdout.flush()


######################################################################################
#
#       SERIALTHREAD
# gestion de la communication avec le moteur
#
######################################################################################
class serialThread(threading.Thread):
    def __init__(self, serialPort, posEvent, endEvent):  # event = objet Event
        threading.Thread.__init__(self)  # = donnée supplémentaire
        self.serialPort = serialPort
        self.posEvent = posEvent  # on garde un accès à l'objet Event
        self.endEvent = endEvent  # on garde un accès à l'objet Event

    def run(self):
        global posX
        global posY
        global posZ
        global zone1
        global zone2
        global start_time
        self.serialPort.flushInput()
        serialString = ""
        serialString = str(self.serialPort.readline())
        self.serialPort.flushInput()
        time.sleep(1)
        print("serialThread start")
        while not self.endEvent.isSet():
            if start_time != 0 : print_there(3, 50, "t: " + str(int(time.time() - start_time)))
            serialString = ""
            if self.serialPort.in_waiting > 0:
                serialString = str(self.serialPort.readline())
                # ~ print("==> " + str(serialString))
                if serialString != "":
                    serialString = serialString[2:-5]
                    print_there(1, 50, serialString + ' ' * (30 - len(serialString)))
                    if serialString != "":
                        if serialString[-2:] == ":1" and not posEvent.isSet():
                            self.posEvent.set()
                        pos = str(serialString).split(":")
                        if len(pos) == 4:
                            posX = float(pos[0])
                            posY = float(pos[1])
                            posZ = float(pos[2])
                            print_there(2, 50, "X: " + str(posX)+ ' ' * (7 - len(str(posX))))
                            print_there(2, 60, "Y: " + str(posY)+ ' ' * (7 - len(str(posY))))
                            print_there(2, 70, "Z: " + str(posZ)+ ' ' * (7 - len(str(posZ))))


######################################################################################
#
#       DETECTTHREAD
# gestion de la communication avec la carte de detection
#
######################################################################################
class detectThread(threading.Thread):
    def __init__(self, serialPort, detectEvent, endEvent):  # event = objet Event
        threading.Thread.__init__(self)  # = donnée supplémentaire
        self.serialPort = serialPort
        self.detectEvent = detectEvent  # on garde un accès à l'objet Event
        self.endEvent = endEvent  # on garde un accès à l'objet Event

    def run(self):
        try:
            detectPort = serial.Serial(port="/dev/USB_DETECT", baudrate=9600, timeout=1, writeTimeout=1)
            detectPort.flushInput()
            serialString = ""
            serialString = str(detectPort.readline())
            detectPort.flushInput()
            # ~ time.sleep(1)
            print("detectThread start")
            while not self.endEvent.isSet():
                if detectPort.in_waiting > 0:
                    serialString = str(detectPort.readline())
                    detectPort.flushInput()
                    # ~ print("==> " + str(serialString))
                    if serialString != "":
                        serialString = serialString[2:-5]
                        print_there(3, 50, "detect: " +serialString)
                        if serialString != "" and serialString != "0":
                            print("stop ")
                            self.detectEvent.set()
                            self.serialPort.write("0\r\n".encode())  # signal de stop                            
                        else:
                            if self.detectEvent.isSet():
                                print("go ")
                                self.serialPort.write(lastV.encode())
                                self.serialPort.write("\r\n".encode())
                                self.detectEvent.clear()
                                
        except:
            print("Erreur ouverture du port serie detection.")
        finally:
            detectPort.close()
            

######################################################################################
#
#       LIDARTHREAD
# gestion du lidar
#
######################################################################################
class lidarThread(threading.Thread):
    def __init__(self, serialPort, stopEvent, endEvent):  # event = objet Event
        threading.Thread.__init__(self)  # = donnée supplémentaire
        self.serialPort = serialPort
        self.stopEvent = stopEvent  # on garde un accès à l'objet Event
        self.endEvent = endEvent  # on garde un accès à l'objet Event

    def run(self):
        global posX
        global posY
        global posZ
        global targX
        global targY
        global lastV
        global cote
        global runningMatch

        try:
            minX = 0
            minY = 0
            lidar = RPLidar('/dev/USB_LIDAR')
            print("lidarThread start")
            if debugCV:
                imgBase = np.zeros((200 * affCoef, 300 * affCoef, 3), np.uint8)
                # ~ imgBase = cv2.imread("fond"+str(affCoef)+".png")
                cv2.imshow('frame', imgBase)
                cv2.moveWindow('frame', 400, 150)
                print("imshow")

            for scan in lidar.iter_scans():
                if debugCV and not runningMatch:
                    # Create a black image
                    # (300 x 200)
                    imgBase = np.zeros((200 * affCoef, 300 * affCoef, 3), np.uint8)

                if not self.endEvent.isSet():
                    if debugCV: img = cv2.circle(imgBase.copy(), (int(posX * affCoef), int((200 - posY) * affCoef)),
                                                 affSize * affCoef, (0, 255, 0), -1)

                    # on calcul les parametres de trajectoire
                    a = (targY - posY)
                    b = (posX - targX)
                    c = -(b * posY + a * posX)
                    #distance entre le robot et la cible
                    distT = sqrt((targX - posX) * (targX - posX) + (targY - posY) * (targY - posY))

                    # print(a, b, c, distT, (distT + alertDist))
                    minDistance = 300
                    minAngle = 0
                    mindistP = 300
                    for (_, angle, distance) in scan:
                        if distance > 0 :  # ignore initially ungathered data points
                            # ~ radians = (angle - posZ - offsetAngle) * pi / 180.0
                            radians = (posZ - angle - offsetAngle) * pi / 180.0
                            x = distance * 0.1 * cos(radians) + posX
                            y = distance * 0.1 * sin(radians) + posY
                            if 0 < int(x) < 300 and 0 < int(y) < 200:
                                if debugCV:
                                    # ~ img[int((200 - y) * affCoef), int(x * affCoef)] = [255, 255, 0]
                                    img[int((200-y) * affCoef), int(x * affCoef)] = [255, 255, 0]
                                    # ~ if runningMatch: imgBase[int((200 - y) * affCoef), int(x * affCoef)] = [0, 0, 255]
                                    if runningMatch: imgBase[int((200-y) * affCoef), int(x * affCoef)] = [0, 0, 255]
                                
                                if distance > 0 and (a * a + b * b) != 0:
                                    # distance normale à la trajectoire
                                    distN = abs(a * x + b * y + c) / sqrt(a * a + b * b)
                                    # distance par rapport à la position du robot
                                    distR = sqrt((x - posX) * (x - posX) + (y - posY) * (y - posY))
                                    # distance par rapport à l'arrivé
                                    distA = sqrt((x - targX) * (x - targX) + (y - targY) * (y - targY))
                                    
                                    # dans la zone d'alerte, et entre robot et arrivé, et proche trajectoire
                                    if distR < (distT + alertDist) and distA < distT and distN < alertDist:
                                        mindist = min(distN, distA, distR)
                                        if mindist < minDistance :
                                            minDistance = mindist
                                            minAngle = angle - posZ - offsetAngle
                                            minX = x
                                            minY = y
                    
                    if debugCV:
                        if minDistance < alertDist:
                            cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                     (int(minX * affCoef), int((200 - minY) * affCoef)), (0, 0, 255), 2)
                        else:
                            cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                     (int(minX * affCoef), int((200 - minY) * affCoef)), (0, 255, 0), 2)
                        img = cv2.circle(img, (int(minX * affCoef), int((200 - minY) * affCoef)), alertDist * affCoef,
                                         (0, 0, 255), 1)

                    if minDistance < alertDist and runningMatch and not debug:
                        print('Stop urg.')
                        self.serialPort.write("0\r\n".encode())  # signal de stop
                        self.stopEvent.set()
                    else:
                        if self.stopEvent.isSet():
                            print('lastV2')
                            print(lastV)
                            serialPort.write(lastV.encode())
                            serialPort.write("\r\n".encode())
                            self.stopEvent.clear()

                    radians = posZ * pi / 180.0
                    # trajectoire
                    if debugCV: cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                         (int(targX * affCoef), int((200 - targY) * affCoef)),
                                         (0, 255, 255), 2)

                    # l'orientation
                    if debugCV: cv2.line(img, (int(posX * affCoef), int((200 - posY) * affCoef)),
                                         (int((posX + affSize * cos(radians)) * affCoef), int((200 - (posY + affSize * sin(radians))) * affCoef)),
                                         (255, 0, 0), 2)
                    if debugCV: cv2.imshow('frame', img)
                    if debugCV: cv2.waitKey(1)

                else:
                    break

        except KeyboardInterrupt:
            print('Stoping.')
        finally:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        # on arret le lidar


        
######################################################################################
#
#       SENDCMD
# envoi des commandes à la carte moteur
#
######################################################################################
def sendCmd(cmd):
    # on enregistre la cible pour l'evitement
    global targX
    global targY
    global lastV
    pos = cmd.split(",")
    if len(pos) > 1:
        if pos[0] in ["G", "T", "S"]:
            targX = float(pos[1])
            targY = float(pos[2])
        if pos[0] == "V":
            lastV = cmd
    # ~ print("cmd "+cmd)
    serialPort.write(cmd.encode())
    serialPort.write("\r\n".encode())
    # si on attend une réponse
    if pos[0] in ["G", "T", "R", "C", "F"]:
        time.sleep(0.5)
        posEvent.clear()
        if pos[0] in ["G"]: time.sleep(0.5)
        posEvent.wait(5)
        # si on a eu un obstacle
        if detectEvent.isSet() or stopEvent.isSet():
            # on attend le depart de l'obstacle
            while (detectEvent.isSet() or stopEvent.isSet()) and (time.time() - start_time) < 95:
                time.sleep(0.5)
            # on renvoi la commande de déplacement si on a encore le temps
            if (time.time() - start_time) < 95:
                sendCmd(lastV)

######################################################################################
#
#       EXECUTE
# décodage et execution d'une commande
#
######################################################################################
def execute(current):
    global mainScore
    global zone1
    global zone2
    global zone3

    if current.nodeType == 1:

        if current.nodeName == "echo":
            print(str(int(time.time() - start_time)) + " ==== "+current.attributes['txt'].value + " ==== ")

        if current.nodeName == "cmdMtr":
            print("cmd " + current.attributes['cmd'].value)
            if debug:
                wait = input("==")
            sendCmd(current.attributes['cmd'].value)

        if current.nodeName == "wait":
            print("wait " + current.attributes['t'].value)
            if debug:
                chrono_time = time.time()
                wait = input("==")
                print("chrono: " + str(time.time() - chrono_time))
            else:
                time.sleep(float(current.attributes['t'].value))

        if current.nodeName == "waitTo" and not debug:
            print("waitTo " + current.attributes['t'].value)
            while (time.time() - start_time) < float(current.attributes['t'].value):
                time.sleep(1)
    
                
######################################################################################
#
#       MAIN
#
######################################################################################
if __name__ == '__main__':
    
    try:
        serialPort = serial.Serial(port="/dev/USB_MOTOR", baudrate=115200, timeout=1, writeTimeout=1)

    except:
        print("Erreur ouverture du port serie.")
        serialPort.close()
        exit()

    posEvent = threading.Event()  # on crée un objet de type Event
    posEvent.clear()  # on désactive l'ojet Event
    endEvent = threading.Event()  # on crée un objet de type Event
    endEvent.clear()  # on désactive l'ojet Event
    detectEvent = threading.Event()  # on crée un objet de type Event
    detectEvent.clear()  # on désactive l'ojet Event
    stopEvent = threading.Event()  # on crée un objet de type Event
    stopEvent.clear()  # on désactive l'ojet Event


    m = serialThread(serialPort, posEvent, endEvent)  # crée un thread
    m.start()  # démarre le thread,

    # ~ d = detectThread(serialPort, detectEvent, endEvent)  # crée un thread
    # ~ d.start()  # démarre le thread,

    l = lidarThread(serialPort, stopEvent, endEvent)  # crée un thread
    l.start()  # démarre le thread,.
            
    tirette = Button(22)
    cote_G = Button(23)
    cote_D = Button(24)

    from xml.dom import minidom

    try:
        doc = minidom.parse('/home/pi/robot_en_carton/sequence_0.xml')
    except:
        print("Erreur lecture du fichier xml.")
        sys.exit()

    # root = doc.documentElement
    print("fichier sequence_0.xml charger")
    print("")
    # cmd : on coupe les moteurs
    # ~ sendCmd("0")

    if tirette.is_pressed == True and not debug:
        print("Envelez la tirette.")
        # ~ tirette.wait_for_press()
        tirette.wait_for_release()
        print("")
    # ~ time.sleep(4)

    print("1 : choisir le cote du match")
    print("2 : verifier le BAU")
    print("3 : inserer la tirette")
    print("")

    sequencelist = doc.getElementsByTagName('sequence')
    try:
        # on attend la tirette
        # tirette.wait_for_release()
        while tirette.is_pressed == False:
            cote = ""
            time.sleep(0.5)
            if cote_G.is_pressed == False:
                cote = "G"

            if cote_D.is_pressed == False:
                cote = "D"

        # on regarde quel cote a ete selectionne
        cote = ""
        if cote_G.is_pressed == True:
            cote = "G"
            print("G")

        if cote_D.is_pressed == True:
            cote = "D"
            print("D")

        # preparation
        print("===== PREPA =====")
        for s in sequencelist:

            if s.attributes['name'].value == "prep_" + cote:
                current = s.firstChild
                while current:
                    execute(current)
                    current = current.nextSibling

        print("PRET POUR MATCH")
        # on attend la tirette
        # ~ tirette.wait_for_press()
        tirette.wait_for_release()
        runningMatch = True
        
        # match
        print("===== MATCH =====")
        start_time = time.time()
        for s in sequencelist:
            if s.attributes['name'].value == "match_" + cote:
                current = s.firstChild
                while current:
                    execute(current)
                    current = current.nextSibling
                    if (time.time() - start_time) > 95 and not debug:
                        current = ""

        # fin
        print(time.time() - start_time)
        print("===== FIN =====")
        for s in sequencelist:
            if s.attributes['name'].value == "fin_" + cote:
                current = s.firstChild
                while current:
                    execute(current)
                    current = current.nextSibling
                    if (time.time() - start_time) > 100 and not debug:
                        current = ""

        print("===== FIN DE MATCH =====")
    except KeyboardInterrupt:
        print('Interruption')
        
    finally:
        endEvent.set()
    # cmd : on coupe les moteurs
        printScore.printScore(" " + str(mainScore) + "p")
        runningMatch = False
        sendCmd("0")
        sendCmd("0")
        sendCmd("0")
        if debugCV: cv2.destroyAllWindows()
        serialPort.close()
        
        wait = input("Press Enter to continue.")

        
