#!/home/pi/src/venv/bin/python3

from gpiozero import Button

import serial
import time
import sys
import threading

posX = 0.0
posY = 0.0
posZ = 0.0
start_time = 0
mainScore = 0

runningMatch = False
debug = False

cote = ""

def print_there(x, y, text):
    sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
    sys.stdout.flush()

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
                #                 print("==> " + str(serialString))
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


def sendCmd(cmd):
    serialPort.write(cmd.encode())
    serialPort.write("\r\n".encode())
    time.sleep(0.5)
    posEvent.clear()
    posEvent.wait(5)
    

def execute(current):
    global mainScore
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
try:
    serialPort = serial.Serial(port="/dev/USB_MOTOR", baudrate=115200, timeout=1, writeTimeout=1)

except:
    print("Erreur ouverture du port serie.")
    sys.exit()

posEvent = threading.Event()  # on crée un objet de type Event
posEvent.clear()  # on désactive l'ojet Event
endEvent = threading.Event()  # on crée un objet de type Event
endEvent.clear()  # on désactive l'ojet Event

m = serialThread(serialPort, posEvent, endEvent)  # crée un thread
m.start()  # démarre le thread,

tirette = Button(22)
cote_G = Button(23)
cote_D = Button(24)

from xml.dom import minidom

try:
    doc = minidom.parse('/home/pi/src/sequence_0.xml')
except:
    print("Erreur lecture du fichier xml.")
    sys.exit()

# root = doc.documentElement
print("fichier sequence_0.xml charger")
print("")
# cmd : on coupe les moteurs
sendCmd("0")

if tirette.is_pressed == False and not debug:
    print("Envelez la tirette.")
    tirette.wait_for_press()
    print("")
time.sleep(4)

print("1 : choisir le cote du match")
print("2 : verifier le BAU")
print("3 : inserer la tirette")
print("")

sequencelist = doc.getElementsByTagName('sequence')
try:
    # on attend la tirette
    # tirette.wait_for_release()
    while tirette.is_pressed == True:
        cote = ""
        if cote_G.is_pressed == False:
            cote = "G"

        if cote_D.is_pressed == False:
            cote = "D"

    # on regarde quel cote a ete selectionne
    cote = ""
    if cote_G.is_pressed == False:
        cote = "G"
        print("G")

    if cote_D.is_pressed == False:
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
    tirette.wait_for_press()
    
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
# cmd : on coupe les moteurs
    sendCmd("0")
    sendCmd("0")
    sendCmd("0")
    endEvent.set()
    
    wait = input("Press Enter to continue.")

    serialPort.close()
    
